#!/usr/bin/env python3
"""
Build a histogram of point distance (range) from a folder of PCD files,
aggregated over *all files*, and save to CSV.

Assumes PCDs can be read by pypcd4 (including binary / binary_compressed).

Example:
  python3 pcd_range_hist.py \
    --pcd_dir /path/to/pcds \
    --rmin 0.0 --rmax 200.0 --bins 400 \
    --out range_hist.csv

Optional:
  --recursive   # search subfolders
"""

from __future__ import annotations

import argparse
import csv
import os
import re
from pathlib import Path
from typing import List, Tuple

import numpy as np
import pypcd4 as pypcd


def natural_key(s: str):
    # "file_2.pcd" < "file_10.pcd"
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", s)]


def list_pcd_files(pcd_dir: Path, recursive: bool) -> List[Path]:
    if recursive:
        files = [p for p in pcd_dir.rglob("*.pcd") if p.is_file()]
    else:
        files = [p for p in pcd_dir.glob("*.pcd") if p.is_file()]
    files.sort(key=lambda p: natural_key(str(p)))
    return files


def load_pcd_xyz(pcd_path: Path) -> np.ndarray:
    """
    Returns:
      xyz: (N, 3) float64 numpy array
    Robust to small pypcd4 API differences across versions.
    """
    # Try common constructors
    if hasattr(pypcd.PointCloud, "from_path"):
        pc = pypcd.PointCloud.from_path(str(pcd_path))
    elif hasattr(pypcd.PointCloud, "from_file"):
        pc = pypcd.PointCloud.from_file(str(pcd_path))
    else:
        # Some versions expose read_point_cloud()
        if hasattr(pypcd, "read_point_cloud"):
            pc = pypcd.read_point_cloud(str(pcd_path))
        else:
            raise RuntimeError("Unsupported pypcd4 API: cannot find a reader")

    # Extract structured array
    arr = None
    for attr in ("pc_data", "data", "points"):
        if hasattr(pc, attr):
            arr = getattr(pc, attr)
            break
    if arr is None:
        # Some versions store dict-like
        if isinstance(pc, dict) and ("pc_data" in pc or "data" in pc):
            arr = pc.get("pc_data", pc.get("data"))
        else:
            raise RuntimeError(f"Could not extract point array from {pcd_path}")

    if not hasattr(arr, "dtype") or arr.dtype.names is None:
        raise RuntimeError(f"Unexpected point array format in {pcd_path}: dtype={getattr(arr,'dtype',None)}")

    names = set(arr.dtype.names)
    if not {"x", "y", "z"}.issubset(names):
        raise RuntimeError(f"PCD missing x/y/z fields: {pcd_path} has {sorted(names)}")

    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64, copy=False)
    return xyz


def save_histogram_csv(out_csv: str, counts: np.ndarray, edges: np.ndarray) -> None:
    total = int(counts.sum())
    prob = counts / total if total > 0 else np.zeros_like(counts, dtype=np.float64)
    cumprob = np.cumsum(prob)
    centers = 0.5 * (edges[:-1] + edges[1:])

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["bin_left", "bin_right", "bin_center", "count", "probability", "cumulative_probability"]
        )
        for i in range(counts.size):
            w.writerow(
                [
                    float(edges[i]),
                    float(edges[i + 1]),
                    float(centers[i]),
                    int(counts[i]),
                    float(prob[i]),
                    float(cumprob[i]),
                ]
            )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd_dir", help="Folder containing .pcd files")
    ap.add_argument("out", help="Output CSV path")
    ap.add_argument("--recursive", action="store_true", help="Search subfolders too")
    ap.add_argument("--rmin", type=float, default=0.0, help="Min range (meters)")
    ap.add_argument("--rmax", type=float, default=200.0, help="Max range (meters)")
    ap.add_argument("--bins", type=int, default=400, help="Number of histogram bins")

    args = ap.parse_args()

    pcd_dir = Path(args.pcd_dir)
    if not pcd_dir.exists():
        raise FileNotFoundError(f"pcd_dir not found: {pcd_dir}")
    if args.rmax <= args.rmin:
        raise ValueError(f"Invalid range: rmax ({args.rmax}) must be > rmin ({args.rmin})")

    files = list_pcd_files(pcd_dir, args.recursive)
    if not files:
        raise RuntimeError(f"No .pcd files found in: {pcd_dir} (recursive={args.recursive})")

    edges = np.linspace(args.rmin, args.rmax, args.bins + 1, dtype=np.float64)
    counts = np.zeros(args.bins, dtype=np.int64)

    n_files = 0
    n_pts_used = 0
    n_pts_total = 0

    for p in files:
        xyz = load_pcd_xyz(p)

        # filter non-finite
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        n_pts_total += int(finite.sum())

        if xyz.size == 0:
            n_files += 1
            continue

        r = np.linalg.norm(xyz, axis=1)

        # keep in [rmin, rmax)
        mask = (r >= args.rmin) & (r < args.rmax)
        r = r[mask]
        n_pts_used += int(r.size)

        if r.size > 0:
            h, _ = np.histogram(r, bins=edges)
            counts += h.astype(np.int64, copy=False)

        n_files += 1

    save_histogram_csv(args.out, counts, edges)

    print(f"PCD dir: {pcd_dir}")
    print(f"Files found:      {len(files)}")
    print(f"Files processed:  {n_files}")
    print(f"Points total:     {n_pts_total}")
    print(f"Points used:      {n_pts_used}")
    print(f"Histogram total:  {int(counts.sum())}")
    print(f"Saved CSV:        {args.out}")


if __name__ == "__main__":
    main()
