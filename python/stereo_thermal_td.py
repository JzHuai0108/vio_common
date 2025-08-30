#!/usr/bin/env python3
# compute the median of the time diff between left and right thermal image host times

import sys, os, math
from bisect import bisect_left
from decimal import Decimal, getcontext

getcontext().prec = 40  # enough precision for ns math

def parse_host_times(csv_path):
    host_ns = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            # allow comma or whitespace
            parts = [p for p in s.replace(',', ' ').split() if p]
            if len(parts) < 1:
                continue
            host_sec = parts[0]
            # Convert seconds with 9 fractional digits to integer nanoseconds
            d = Decimal(host_sec)
            ns = int((d * Decimal(1_000_000_000)).to_integral_value(rounding=getcontext().rounding))
            host_ns.append(ns)
    return host_ns

def median(nums):
    n = len(nums)
    if n == 0:
        return None
    nums_sorted = sorted(nums)
    mid = n // 2
    if n % 2 == 1:
        return Decimal(nums_sorted[mid])
    else:
        return (Decimal(nums_sorted[mid-1]) + Decimal(nums_sorted[mid])) / Decimal(2)

def nearest(right_sorted_ns, x):
    """Return the closest value in right_sorted_ns to x."""
    i = bisect_left(right_sorted_ns, x)
    if i == 0:
        return right_sorted_ns[0]
    if i == len(right_sorted_ns):
        return right_sorted_ns[-1]
    before = right_sorted_ns[i-1]
    after = right_sorted_ns[i]
    return after if (after - x) < (x - before) else before

def main():
    if len(sys.argv) < 2:
        print("Usage: python sync_time_median.py <seqdir>", file=sys.stderr)
        sys.exit(1)

    seqdir = sys.argv[1]
    left_csv  = os.path.join(seqdir, "left_thermal",  "timestamps.csv")
    right_csv = os.path.join(seqdir, "right_thermal", "timestamps.csv")

    if not os.path.isfile(left_csv):
        print(f"Error: {left_csv} not found", file=sys.stderr); sys.exit(2)
    if not os.path.isfile(right_csv):
        print(f"Error: {right_csv} not found", file=sys.stderr); sys.exit(2)

    left_ns  = parse_host_times(left_csv)
    right_ns = parse_host_times(right_csv)

    if not left_ns or not right_ns:
        print("Error: one of the CSV files has no parsable host timestamps.", file=sys.stderr)
        sys.exit(3)

    right_ns_sorted = sorted(right_ns)

    diffs_ns = []
    for ln in left_ns:
        rn = nearest(right_ns_sorted, ln)
        diffs_ns.append(rn - ln)  # right - left

    med_ns = median(diffs_ns)
    if med_ns is None:
        print("Error: no diffs computed.", file=sys.stderr)
        sys.exit(4)

    # Report in seconds with 9 decimal places
    med_sec = (med_ns / Decimal(1_000_000_000))
    print(f"{med_sec:.9f}")

if __name__ == "__main__":
    main()
