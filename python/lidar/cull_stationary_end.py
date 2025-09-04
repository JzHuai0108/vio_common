#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# cull the stationary end of a record seq with stereo cameras, RGBD, and lidar data.

import argparse
import csv
import datetime as dt
import os
import shutil
import sys
from pathlib import Path
from typing import Iterable, Tuple, Optional

# --------------------------
# Parsing helpers
# --------------------------

def parse_endtime(endtime_str: str) -> float:
    """
    Parse endtime as epoch seconds.
    Accepts:
      - float/int epoch seconds
      - ISO-8601 like '2025-09-02T17:22:54Z' or with offset
    """
    s = endtime_str.strip()
    # Try numeric epoch first
    try:
        return float(s)
    except ValueError:
        pass

    # Normalize a trailing 'Z' to +00:00 for fromisoformat
    if s.endswith("Z"):
        s = s[:-1] + "+00:00"
    try:
        t = dt.datetime.fromisoformat(s)
        if t.tzinfo is None:
            # Assume UTC if no timezone provided (your note said UTC time)
            t = t.replace(tzinfo=dt.timezone.utc)
        return t.timestamp()
    except Exception as e:
        raise ValueError(f"Failed to parse endtime '{endtime_str}': {e}")


def stamp_from_filename(fname: str) -> Optional[float]:
    """
    Extract epoch seconds from a filename like 'sec.nsec.png' or 'sec.nsec.ext'.
    Returns None if the pattern doesn't match.
    """
    stem = Path(fname).stem  # drops extension(s)
    # If multiple dots (e.g., .png), get the last two numeric parts
    parts = stem.split(".")
    if len(parts) < 2:
        return None
    sec_str, nsec_str = parts[0], parts[1]
    try:
        sec = int(sec_str)
        # nsec may have no leading zeros; clamp to [0, 1e9)
        nsec = int(nsec_str)
        if nsec < 0:
            return None
        if nsec >= 1_000_000_000:
            # Handle occasional micro/milli mixups by truncating to 9 digits
            nsec = int(str(nsec)[:9])
        return sec + nsec * 1e-9
    except ValueError:
        return None


# --------------------------
# Filesystem trimming
# --------------------------

def trim_images_in_dir(dirpath: Path, end_ts: float, dry_run: bool = False) -> Tuple[int, int]:
    """
    Remove files with filename timestamps > end_ts in dirpath.
    Returns (kept, removed).
    """
    if not dirpath.exists():
        return (0, 0)
    kept, removed = 0, 0
    for p in sorted(dirpath.iterdir()):
        if not p.is_file():
            continue
        ts = stamp_from_filename(p.name)
        if ts is None:
            kept += 1
            continue
        if ts > end_ts:
            removed += 1
            if not dry_run:
                try:
                    p.unlink()
                except Exception as e:
                    print(f"[WARN] Failed to remove {p}: {e}", file=sys.stderr)
        else:
            kept += 1
    return kept, removed


def _parse_float_or_skip(s: str) -> Optional[float]:
    try:
        return float(s.strip())
    except Exception:
        return None


def trim_timestamps_csv(csv_path: Path, end_ts: float, host_col_index: int = 0, delimiter: Optional[str] = None,
                        dry_run: bool = False) -> Tuple[int, int]:
    """
    Trim rows where host time (column host_col_index) > end_ts.
    Preserves:
      - lines starting with '#'
      - a single header row if first data line cannot be parsed as float
    If delimiter is None, csv.Sniffer is used (fallback to comma).
    Returns (kept_rows, removed_rows) excluding comments.
    """
    if not csv_path.exists():
        return (0, 0)

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        content = f.read()

    lines = content.splitlines()
    if not lines:
        return (0, 0)

    # Detect delimiter if not provided
    if delimiter is None:
        try:
            sniffer = csv.Sniffer()
            dialect = sniffer.sniff(content)
            delim = dialect.delimiter
        except Exception:
            delim = ","
    else:
        delim = delimiter

    out_lines = []
    kept, removed = 0, 0
    header_handled = False

    for line in lines:
        if not line.strip():
            out_lines.append(line)
            continue
        if line.lstrip().startswith("#"):
            out_lines.append(line)
            continue

        # Try split by detected delimiter
        row = [c for c in csv.reader([line], delimiter=delim)][0]
        if not header_handled:
            # Determine if first row is header by checking host_col parse
            host_val = _parse_float_or_skip(row[host_col_index]) if len(row) > host_col_index else None
            if host_val is None:
                # Keep header line as-is
                out_lines.append(line)
                header_handled = True
                continue
            else:
                header_handled = True
                # fall through to normal filtering

        host_val = _parse_float_or_skip(row[host_col_index]) if len(row) > host_col_index else None
        if host_val is None:
            # If unparsable data row, keep it (conservative)
            out_lines.append(line)
            kept += 1
            continue

        if host_val > end_ts:
            removed += 1
        else:
            kept += 1
            out_lines.append(line)

    if not dry_run:
        tmp = csv_path.with_suffix(csv_path.suffix + ".tmp")
        with tmp.open("w", newline="", encoding="utf-8") as f:
            f.write("\n".join(out_lines) + ("\n" if content.endswith("\n") else ""))
        shutil.move(str(tmp), str(csv_path))
    else:
        tmp = csv_path.with_suffix(csv_path.suffix + ".tmp")
        with tmp.open("w", newline="", encoding="utf-8") as f:
            f.write("\n".join(out_lines) + ("\n" if content.endswith("\n") else ""))
    return kept, removed


# --------------------------
# Rosbag trimming (ROS1)
# --------------------------

def trim_rosbag(bag_path: Path, end_ts: float,
                topics_to_trim: Iterable[str] = ("/lidar_points", "/imu/data"),
                overwrite: bool = True, dry_run: bool = False) -> Tuple[int, int, int]:
    """
    Copy bag, dropping messages on topics_to_trim with header.stamp > end_ts.
    Others are copied untouched.
    Returns (written_msgs, dropped_msgs, total_msgs).
    """
    try:
        import rosbag
    except Exception as e:
        raise RuntimeError("rosbag Python API not available. Please run inside a ROS1 environment.") from e

    src = str(bag_path)
    if not os.path.exists(src):
        raise FileNotFoundError(f"Bag not found: {src}")

    dst = src + ".tmp"
    written = 0
    dropped = 0
    total = 0

    def msg_stamp_sec(msg) -> Optional[float]:
        # Expecting msg.header.stamp
        try:
            return msg.header.stamp.to_sec()
        except Exception:
            return None

    with rosbag.Bag(src, "r") as inbag, rosbag.Bag(dst, "w") as outbag:
        for topic, msg, t in inbag.read_messages():
            total += 1
            if topic in topics_to_trim:
                ts = msg_stamp_sec(msg)
                if ts is not None and ts > end_ts:
                    dropped += 1
                    if dry_run:
                        continue
                    else:
                        # Skip writing this one
                        continue
            # write everything else (and trimmed topics when within time)
            written += 1
            if not dry_run:
                outbag.write(topic, msg, t)

    if not dry_run:
        if overwrite:
            # Replace original atomically
            bak = src + ".bak"
            try:
                if os.path.exists(bak):
                    os.remove(bak)
                os.replace(src, bak)
                os.replace(dst, src)
                # optional: keep .bak, or remove it
                os.remove(bak)
            except Exception as e:
                # Attempt to roll back
                print(f"[ERROR] Failed to replace bag: {e}", file=sys.stderr)
                try:
                    if os.path.exists(dst):
                        os.remove(dst)
                except Exception:
                    pass
                raise
        else:
            # Keep tmp as new bag
            pass
    else:
        # Dry run: remove tmp
        try:
            if os.path.exists(dst):
                os.remove(dst)
        except Exception:
            pass

    return written, dropped, total


# --------------------------
# Pipeline for your folder layout
# --------------------------

def run_pipeline(seqdir: Path, end_ts: float,
                 dry_run: bool = False, verbose: bool = True):
    """
    Applies the trimming to:
      - left_thermal/{image,temp_raw} + timestamps.csv
      - right_thermal/{image,temp_raw} + timestamps.csv
      - realsense/{depth_raw,rgb} + timestamps.csv
      - rosbag topics /lidar_points and /imu/data
    """
    def log(msg: str):
        if verbose:
            print(msg)

    # Thermal (left / right)
    for side in ("left_thermal", "right_thermal"):
        base = seqdir / side
        for sub in ("image", "temp_raw"):
            kept, removed = trim_images_in_dir(base / sub, end_ts, dry_run=dry_run)
            log(f"[{side}/{sub}] kept={kept}, removed={removed}")
        # timestamps.csv: host_time, sensor_time (host in col 0)
        kept, removed = trim_timestamps_csv(base / "timestamps.csv", end_ts, host_col_index=0, dry_run=dry_run)
        log(f"[{side}/timestamps.csv] kept_rows={kept}, removed_rows={removed}")

    # Realsense RGBD
    rs_base = seqdir / "realsense"
    for sub in ("depth_raw", "rgb"):
        kept, removed = trim_images_in_dir(rs_base / sub, end_ts, dry_run=dry_run)
        log(f"[realsense/{sub}] kept={kept}, removed={removed}")
    # timestamps.csv: '#host_ts, sensor_ts, now_ts, arrival_ts' (host in col 0, may have '#' header)
    kept, removed = trim_timestamps_csv(rs_base / "timestamps.csv", end_ts, host_col_index=0, dry_run=dry_run)
    log(f"[realsense/timestamps.csv] kept_rows={kept}, removed_rows={removed}")

    # Rosbag
    bag_path = seqdir / "rosbag" / "xt32_d455imu_corrected.bag"
    try:
        written, dropped, total = trim_rosbag(bag_path, end_ts,
                                                topics_to_trim=("/lidar_points", "/imu/data"),
                                                overwrite=True, dry_run=dry_run)
        log(f"[rosbag] total={total}, written={written}, dropped={dropped}, path={bag_path}")
    except Exception as e:
        print(f"[ERROR] rosbag trimming failed: {e}", file=sys.stderr)


# --------------------------
# CLI
# --------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Trim a seq directory (thermal/RGBD) and a ROS1 bag by endtime (UTC)."
    )
    parser.add_argument("seqdir", type=Path, help="Sequence root directory")
    parser.add_argument("endtime", type=str,
                        help="End time (UTC). Accepts epoch seconds or ISO-8601 like 2025-09-02T17:22:54Z")

    parser.add_argument("--dry-run", action="store_true", help="Show what would be removed without changing files")
    parser.add_argument("--quiet", action="store_true", help="Less logging")

    args = parser.parse_args()
    end_ts = parse_endtime(args.endtime)

    run_pipeline(args.seqdir, end_ts, dry_run=args.dry_run, verbose=not args.quiet)


if __name__ == "__main__":
    main()
