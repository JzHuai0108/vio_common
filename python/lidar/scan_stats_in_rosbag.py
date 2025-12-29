#!/usr/bin/env python3
"""
Build a histogram of point distance (range) from a PointCloud2 topic in a ROS1 bag,
aggregated over *all scans*, and save to CSV.

Example:
  python bag_pc2_range_hist.py \
    --bag /path/to/data.bag \
    --topic /rslidar_points \
    --rmin 0.0 --rmax 200.0 --bins 400 \
    --out range_hist.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from typing import Iterable, List, Tuple

import numpy as np

import rosbag
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


def _iter_ranges_from_pc2(
    msg: PointCloud2,
    fields: Tuple[str, str, str] = ("x", "y", "z"),
    chunk_size: int = 200_000,
) -> Iterable[np.ndarray]:
    """
    Yield ranges in chunks (numpy arrays) from a PointCloud2 message.
    Uses read_points() with skip_nans=True, so NaN points are skipped.
    """
    buf: List[float] = []
    # read_points yields tuples (x, y, z) if field_names is set
    for x, y, z in point_cloud2.read_points(msg, field_names=fields, skip_nans=True):
        # Guard against inf (skip_nans doesn't remove inf)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        r = math.sqrt(x * x + y * y + z * z)
        buf.append(r)
        if len(buf) >= chunk_size:
            yield np.asarray(buf, dtype=np.float32)
            buf.clear()

    if buf:
        yield np.asarray(buf, dtype=np.float32)


def build_histogram_from_bag(
    bag_path: str,
    topic: str,
    rmin: float,
    rmax: float,
    bins: int,
    chunk_size: int = 200_000,
) -> Tuple[np.ndarray, np.ndarray, int, int]:
    """
    Returns:
      counts: (bins,) int64
      edges:  (bins+1,) float64
      n_msgs: number of PointCloud2 messages processed
      n_pts:  number of points accumulated (after filtering)
    """
    if rmax <= rmin:
        raise ValueError(f"Invalid range: rmax ({rmax}) must be > rmin ({rmin})")

    edges = np.linspace(rmin, rmax, bins + 1, dtype=np.float64)
    counts = np.zeros(bins, dtype=np.int64)

    n_msgs = 0
    n_pts = 0
    with rosbag.Bag(bag_path, "r") as bag:
        for tp, msg, _ in bag.read_messages():
            if tp != topic:
                continue
            if getattr(msg, "_type", "") != "sensor_msgs/PointCloud2":
                continue

            n_msgs += 1
            for r_chunk in _iter_ranges_from_pc2(msg, chunk_size=chunk_size):
                # Filter by [rmin, rmax)
                mask = (r_chunk >= rmin) & (r_chunk < rmax)
                if not np.any(mask):
                    continue
                r_use = r_chunk[mask]
                n_pts += int(r_use.size)

                # Update histogram
                h, _ = np.histogram(r_use, bins=edges)
                counts += h.astype(np.int64, copy=False)

    return counts, edges, n_msgs, n_pts


def save_histogram_csv(
    out_csv: str,
    counts: np.ndarray,
    edges: np.ndarray,
) -> None:
    """
    CSV columns:
      bin_left, bin_right, bin_center, count, probability, cumulative_probability
    """
    total = int(counts.sum())
    prob = counts / total if total > 0 else np.zeros_like(counts, dtype=np.float64)
    cumprob = np.cumsum(prob)

    centers = 0.5 * (edges[:-1] + edges[1:])

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "bin_left",
                "bin_right",
                "bin_center",
                "count",
                "probability",
                "cumulative_probability",
            ]
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
    ap.add_argument("bag", help="Path to rosbag (*.bag)")
    ap.add_argument("out", help="Output CSV path")
    ap.add_argument("--topic", default="/rslidar_points", help="PointCloud2 topic name")
    ap.add_argument("--rmin", type=float, default=0.0, help="Min range (meters)")
    ap.add_argument("--rmax", type=float, default=200.0, help="Max range (meters)")
    ap.add_argument("--bins", type=int, default=400, help="Number of histogram bins")
    ap.add_argument(
        "--chunk_size",
        type=int,
        default=200_000,
        help="Points per chunk when processing each scan",
    )
    args = ap.parse_args()

    counts, edges, n_msgs, n_pts = build_histogram_from_bag(
        bag_path=args.bag,
        topic=args.topic,
        rmin=args.rmin,
        rmax=args.rmax,
        bins=args.bins,
        chunk_size=args.chunk_size,
    )

    save_histogram_csv(args.out, counts, edges)

    total = int(counts.sum())
    print(f"Bag:   {args.bag}")
    print(f"Topic: {args.topic}")
    print(f"Msgs processed:   {n_msgs}")
    print(f"Points used:      {n_pts}")
    print(f"Histogram total:  {total}")
    print(f"Saved CSV:        {args.out}")


if __name__ == "__main__":
    main()
