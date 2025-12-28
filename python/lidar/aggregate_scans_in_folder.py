#!/usr/bin/env python3
"""
Aggregate a folder of PCL-written binary_compressed PCDs (PointXYZINormal) into one merged PCD.

Inputs:
  1) pcd_dir: contains PCD files named like:
       sec.nsec.pcd   (e.g. 1700000000.123456789.pcd)
     (also accepts sec_nsec.pcd)
  2) states_txt: each line:
       time(sec) x y z qx qy qz qw [others...]
     quaternion is (x,y,z,w). Times must match PCD filenames.

Behavior:
  - Read PCDs with pypcd4 (supports PCL binary_compressed).
  - Extract x,y,z,intensity; if normals exist, rotate and save them too.
  - Transform to world: p_w = R(q) * p_l + t_w.
  - Optional max-range filter (in lidar frame).
  - Save output as PCD via pypcd4 (no save(compression=...) kw).
  - Save per-frame distance stats CSV, and optional global range histogram CSV.
  - All timestamps in CSV are saved as "sec.%09d" (not raw ns int).

Notes:
  - If some frames don't contain normals but others do, missing normals are filled with zeros.
"""

import argparse
import csv
import math
import re
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path
from typing import Dict, Optional, Tuple, List

import numpy as np
from scipy.spatial.transform import Rotation as R

from pypcd4 import PointCloud


# -------------------------
# Time helpers (exact "sec.%09d")
# -------------------------

_TIME_RE_DOT = re.compile(r"^(?P<sec>\d+)\.(?P<nsec>\d+)$")
_TIME_RE_UND = re.compile(r"^(?P<sec>\d+)_(?P<nsec>\d+)$")


def ns_to_timestr(t_ns: int) -> str:
    sec = t_ns // 1_000_000_000
    nsec = t_ns % 1_000_000_000
    return f"{sec}.{nsec:09d}"


def secstr_to_ns_and_timestr(t_sec_str: str) -> Tuple[int, str]:
    """
    Robust parse of a decimal seconds token -> (ns_int, "sec.%09d").
    Avoids float rounding.
    """
    d = Decimal(t_sec_str.strip())
    t_ns = int((d * Decimal(1_000_000_000)).to_integral_value(rounding=ROUND_HALF_UP))
    return t_ns, ns_to_timestr(t_ns)


def parse_time_from_pcd_name(pcd_path: Path) -> Optional[Tuple[int, str]]:
    """
    Parse filename stem:
      - sec.nsec
      - sec_nsec
    Return (t_ns, "sec.%09d") or None.
    """
    stem = pcd_path.stem
    m = _TIME_RE_DOT.match(stem) or _TIME_RE_UND.match(stem)
    if not m:
        return None

    sec = int(m.group("sec"))
    nsec_str = m.group("nsec")

    # Normalize to 9 digits
    if len(nsec_str) < 9:
        nsec = int(nsec_str.ljust(9, "0"))
    else:
        nsec = int(nsec_str[:9])

    t_ns = sec * 1_000_000_000 + nsec
    return t_ns, f"{sec}.{nsec:09d}"


# -------------------------
# States
# -------------------------

@dataclass
class Pose:
    t_ns: int
    t_str: str          # "sec.%09d"
    p: np.ndarray       # (3,) float64
    rot: R              # scipy Rotation (world <- lidar)


def load_states_txt(states_path: Path) -> Dict[int, Pose]:
    """
    Each line: time(sec) x y z qx qy qz qw ...
    Return: {time_ns: Pose}
    """
    states: Dict[int, Pose] = {}
    with states_path.open("r") as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                raise ValueError(f"{states_path}:{ln} expected >=8 columns, got {len(parts)}")

            t_ns, t_str = secstr_to_ns_and_timestr(parts[0])
            x, y, z = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])

            states[t_ns] = Pose(
                t_ns=t_ns,
                t_str=t_str,
                p=np.array([x, y, z], dtype=np.float64),
                rot=R.from_quat([qx, qy, qz, qw]),  # scipy expects (x,y,z,w)
            )
    return states


# -------------------------
# PCD IO (pypcd4)
# -------------------------

def load_pcd_xyz_intensity_normals(pcd_path: Path) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    """
    Load from PCD (binary/binary_compressed) using pypcd4.

    Returns:
      xyz_l: (N,3) float32
      intensity: (N,) float32 (zeros if not present)
      normals_l: (N,3) float32 or None (if normal fields absent)
    """
    pc = PointCloud.from_path(str(pcd_path))
    fields = pc.fields
    data = pc.pc_data

    for req in ("x", "y", "z"):
        if req not in fields:
            raise ValueError(f"{pcd_path} missing field '{req}', fields={fields}")

    xyz = np.stack([
        data["x"].astype(np.float32),
        data["y"].astype(np.float32),
        data["z"].astype(np.float32),
    ], axis=1)

    if "intensity" in fields:
        intensity = data["intensity"].astype(np.float32).reshape(-1)
    else:
        intensity = np.zeros((xyz.shape[0],), dtype=np.float32)

    normals = None
    if all(k in fields for k in ("normal_x", "normal_y", "normal_z")):
        normals = np.stack([
            data["normal_x"].astype(np.float32),
            data["normal_y"].astype(np.float32),
            data["normal_z"].astype(np.float32),
        ], axis=1)

    return xyz, intensity, normals


def make_pointcloud_for_save_xyzi_or_xyzin(
    xyz: np.ndarray,
    intensity: np.ndarray,
    normals: Optional[np.ndarray],
) -> PointCloud:
    """
    Build pypcd4 PointCloud. If normals is provided, save fields:
      x y z intensity normal_x normal_y normal_z
    Otherwise save XYZI.
    """
    xyz = xyz.astype(np.float32, copy=False)
    intensity = intensity.astype(np.float32, copy=False).reshape(-1, 1)

    if normals is not None:
        normals = normals.astype(np.float32, copy=False)
        arr = np.concatenate([xyz, intensity, normals], axis=1)
        fields = ("x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z")
        types = (np.float32,) * 7
        return PointCloud.from_points(arr, fields, types)

    arr = np.concatenate([xyz, intensity], axis=1)
    return PointCloud.from_xyzi_points(arr)


# -------------------------
# Aggregation
# -------------------------

def aggregate_pcd_folder(
    pcd_dir: Path,
    states_txt: Path,
    out_pcd: Path,
    max_range: float = math.inf,
    frame_stride: int = 1,
    save_dist_histo: bool = False,
    histo_bins: int = 200,
    histo_max: Optional[float] = None,
) -> None:
    states = load_states_txt(states_txt)

    pcd_files = sorted(pcd_dir.glob("*.pcd"))
    if not pcd_files:
        raise FileNotFoundError(f"No .pcd files found under: {pcd_dir}")

    # Match PCDs to states by exact ns time
    selected: List[Tuple[int, str, Path]] = []
    skipped_name = 0
    skipped_nomatch = 0
    for p in pcd_files:
        parsed = parse_time_from_pcd_name(p)
        if parsed is None:
            skipped_name += 1
            continue
        t_ns, t_str = parsed
        if t_ns in states:
            selected.append((t_ns, t_str, p))
        else:
            skipped_nomatch += 1

    if not selected:
        raise RuntimeError(
            f"No PCDs matched states times.\n"
            f"pcd_dir={pcd_dir}\nstates={states_txt}\n"
            f"skipped_bad_name={skipped_name}, skipped_no_state_match={skipped_nomatch}"
        )

    selected.sort(key=lambda x: x[0])
    if frame_stride > 1:
        selected = selected[::frame_stride]

    all_xyz_w: List[np.ndarray] = []
    all_int: List[np.ndarray] = []
    all_n_w: List[np.ndarray] = []
    have_any_normals = False

    # Histo/stat collectors
    global_ranges: List[np.ndarray] = []
    frame_stats_rows: List[dict] = []

    total_raw_pts = 0
    total_used_pts = 0

    for idx, (t_ns, t_str_from_name, pcd_path) in enumerate(selected, 1):
        pose = states[t_ns]  # Pose stores exact t_str from states
        # Prefer states time string (since you asked for exact time from states too)
        t_str = pose.t_str

        xyz_l, intensity, normals_l = load_pcd_xyz_intensity_normals(pcd_path)
        total_raw_pts += xyz_l.shape[0]

        ranges = np.linalg.norm(xyz_l.astype(np.float64), axis=1)

        if math.isfinite(max_range):
            keep = ranges <= float(max_range)
            xyz_l = xyz_l[keep]
            intensity = intensity[keep]
            ranges = ranges[keep]
            if normals_l is not None:
                normals_l = normals_l[keep]

        n_used = int(xyz_l.shape[0])
        total_used_pts += n_used

        if save_dist_histo:
            global_ranges.append(ranges.astype(np.float32, copy=False))

        frame_stats_rows.append({
            "time": t_str,  # "sec.%09d"
            "pcd": pcd_path.name,
            "num_points": n_used,
            "range_mean": float(np.mean(ranges)) if ranges.size else 0.0,
            "range_median": float(np.median(ranges)) if ranges.size else 0.0,
            "range_max": float(np.max(ranges)) if ranges.size else 0.0,
        })

        if n_used == 0:
            continue

        R_wl = pose.rot.as_matrix().astype(np.float64)
        xyz_w = (xyz_l.astype(np.float64) @ R_wl.T) + pose.p.reshape(1, 3)
        all_xyz_w.append(xyz_w.astype(np.float32))
        all_int.append(intensity.astype(np.float32))

        if normals_l is not None:
            have_any_normals = True
            n_w = (normals_l.astype(np.float64) @ R_wl.T).astype(np.float32)
            all_n_w.append(n_w)
        elif have_any_normals:
            # If earlier frames had normals but this one doesn't, keep schema consistent
            all_n_w.append(np.zeros((n_used, 3), dtype=np.float32))

        if idx % 50 == 0 or idx == len(selected):
            print(f"[{idx}/{len(selected)}] processed {pcd_path.name}, used_pts={n_used}")

    if not all_xyz_w:
        raise RuntimeError("No points kept after filtering (max-range too small or empty PCDs).")

    xyz_w_all = np.concatenate(all_xyz_w, axis=0)
    int_all = np.concatenate(all_int, axis=0)

    normals_all = None
    if have_any_normals:
        normals_all = np.concatenate(all_n_w, axis=0)
        if normals_all.shape[0] != xyz_w_all.shape[0]:
            raise RuntimeError("Normals/points size mismatch after concatenation.")

    # pc_out = make_pointcloud_for_save_xyzi_or_xyzin(xyz_w_all, int_all, normals_all)
    pc_out = make_pointcloud_for_save_xyzi_or_xyzin(xyz_w_all, int_all, None)

    out_pcd.parent.mkdir(parents=True, exist_ok=True)
    pc_out.save(str(out_pcd))  # IMPORTANT: pypcd4.save() has no `compression=` kw

    print(f"[OK] Saved aggregated PCD: {out_pcd}")
    print(f"     frames_used={len(selected)} raw_pts={total_raw_pts} used_pts={total_used_pts} merged_pts={xyz_w_all.shape[0]}")
    print(f"     normals_saved={bool(normals_all is not None)}")

    # Per-frame stats CSV (always written; small and useful)
    stats_csv = out_pcd.with_suffix("").as_posix() + "_frame_stats.csv"
    with open(stats_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "pcd", "num_points", "range_mean", "range_median", "range_max"])
        writer.writeheader()
        writer.writerows(frame_stats_rows)
    print(f"[OK] Saved per-frame stats: {stats_csv}")

    # Global range histogram (optional)
    if save_dist_histo:
        all_r = np.concatenate(global_ranges, axis=0) if global_ranges else np.array([], dtype=np.float32)
        if all_r.size == 0:
            print("[WARN] No ranges collected for histogram.")
            return

        if histo_max is None:
            if math.isfinite(max_range):
                histo_max = float(max_range)
            else:
                histo_max = float(np.max(all_r))
        histo_max = max(float(histo_max), 1e-6)

        counts, edges = np.histogram(all_r, bins=int(histo_bins), range=(0.0, histo_max))
        hist_csv = out_pcd.with_suffix("").as_posix() + "_range_hist.csv"
        with open(hist_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["bin_left", "bin_right", "count"])
            for i in range(len(counts)):
                w.writerow([float(edges[i]), float(edges[i + 1]), int(counts[i])])
        print(f"[OK] Saved range histogram: {hist_csv}")


# -------------------------
# CLI
# -------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Aggregate a folder of PCL binary_compressed PCDs into one world-frame PCD using poses from states.txt."
    )
    ap.add_argument("pcd_dir", type=Path,
                    help="Input dir containing PCD files named like sec.nsec.pcd (or sec_nsec.pcd).")
    ap.add_argument("states_txt", type=Path,
                    help="States file: time(sec) x y z qx qy qz qw ... (quat is x y z w).")
    ap.add_argument("out_pcd", type=Path,
                    help="Output aggregated PCD path.")
    ap.add_argument("--max-range", type=float, default=math.inf,
                    help="Keep points with ||p_lidar|| <= max-range (default: inf).")
    ap.add_argument("--frame-stride", type=int, default=1,
                    help="Use every N-th matched frame (default: 1).")
    ap.add_argument("--save-dist-histo", action="store_true",
                    help="Also save global range histogram CSV (and always saves per-frame stats CSV).")
    ap.add_argument("--histo-bins", type=int, default=200,
                    help="Histogram bin count (default: 200).")
    ap.add_argument("--histo-max", type=float, default=None,
                    help="Histogram max range (default: max-range if set else max observed).")

    args = ap.parse_args()

    aggregate_pcd_folder(
        pcd_dir=args.pcd_dir,
        states_txt=args.states_txt,
        out_pcd=args.out_pcd,
        max_range=args.max_range,
        frame_stride=max(1, args.frame_stride),
        save_dist_histo=args.save_dist_histo,
        histo_bins=max(1, args.histo_bins),
        histo_max=args.histo_max,
    )


if __name__ == "__main__":
    main()
