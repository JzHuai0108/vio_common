#!/usr/bin/env python3
import sys
import math
import argparse
from typing import List, Tuple, Optional

import numpy as np
try:
    from scipy.spatial.transform import Rotation as Rot
except Exception as e:
    print("Error: scipy is required. Install with `pip install scipy`.", file=sys.stderr)
    raise

# Field index layout (0-based, end-exclusive slices)
IDX_P      = slice(1, 4)    # pxyz (world)
IDX_Q      = slice(4, 8)    # qxyzw (world->body)
IDX_V      = slice(8, 11)   # vxyz (world)
IDX_BG     = slice(11, 14)  # bg (body)  -- unchanged
IDX_BA     = slice(14, 17)  # ba (body)  -- unchanged
IDX_G      = slice(17, 20)  # gravity (world)
IDX_EXT_P  = slice(20, 23)  # extrinsic p  -- unchanged
IDX_EXT_Q  = slice(23, 27)  # extrinsic q  -- unchanged

AFFECTED_SLICES = [IDX_P, IDX_Q, IDX_V, IDX_G]  # only these are rewritten

def parse_line_tokens(line: str) -> Optional[List[str]]:
    """Split a line to tokens; ensure at least 27 numeric fields exist."""
    parts = line.strip().split()
    if len(parts) < 27:
        return None
    return parts[:27]

def tokens_to_floats(tokens: List[str], sl: slice) -> np.ndarray:
    return np.array([float(tokens[i]) for i in range(sl.start, sl.stop)], dtype=float)

def set_tokens_from_array(tokens: List[str], sl: slice, arr: np.ndarray, is_quat=False):
    """Write arr back into tokens with required formatting."""
    fmt = (lambda x: f"{x:.9f}") if is_quat else (lambda x: f"{x:.6f}")
    for k, i in enumerate(range(sl.start, sl.stop)):
        tokens[i] = fmt(float(arr[k]))

def world_to_zup_rotation(g_world: np.ndarray) -> Rot:
    """Return Rot that maps world vectors into Wzup where +Z aligns with -g."""
    g_norm = np.linalg.norm(g_world)
    if g_norm < 1e-8:
        return Rot.identity()
    a = (-g_world / g_norm)            # desired to become +Z
    b = np.array([0.0, 0.0, 1.0])      # +Z
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if c > 0.999999:                   # already aligned
        return Rot.identity()
    axis = np.cross(a, b)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-12:
        # 180°: choose any axis orthogonal to a
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis = axis - a * np.dot(a, axis)
        axis /= np.linalg.norm(axis)
        angle = math.pi
    else:
        axis /= axis_norm
        angle = math.acos(c)
    return Rot.from_rotvec(axis * angle)

def main():
    ap = argparse.ArgumentParser(description="Rotate trajectory to Z-up world (Z along -gravity).")
    ap.add_argument("infile", type=str, help="Input traj txt")
    ap.add_argument("outfile", type=str, help="Output traj txt (Z-up)")
    ap.add_argument(
        "--gravity_from_start",
        type=float,
        nargs="?",
        default=5.0,
        help="Time offset in seconds from the start of the trajectory to select the gravity vector (default: 5.0)."
    )
    args = ap.parse_args()

    raw_lines: List[str] = []
    token_lines: List[Optional[List[str]]] = []
    with open(args.infile, "r") as f:
        for line in f:
            raw_lines.append(line.rstrip("\n"))
            token_lines.append(parse_line_tokens(line))

    # Build (t, tokens) of valid data rows
    entries: List[Tuple[float, List[str]]] = []
    for toks in token_lines:
        if toks is None:
            continue
        try:
            t = float(toks[0])
            entries.append((t, toks))
        except ValueError:
            continue

    if not entries:
        print("No valid data lines found. Exiting.", file=sys.stderr)
        sys.exit(1)

    # gravity reference at ~t0+10s
    t0 = entries[0][0]
    t_target = t0 + args.gravity_from_start
    idx_10 = next((i for i, (t, _) in enumerate(entries) if t >= t_target), len(entries) - 1)
    g_world_10 = tokens_to_floats(entries[idx_10][1], IDX_G)

    Rzup_W: Rot = world_to_zup_rotation(g_world_10)

    out_lines: List[str] = []
    for raw, toks in zip(raw_lines, token_lines):
        if toks is None:
            # Non-data or short lines: pass through unchanged
            out_lines.append(raw)
            continue

        # Read affected fields as floats
        p = tokens_to_floats(toks, IDX_P)
        q = tokens_to_floats(toks, IDX_Q)  # xyzw
        v = tokens_to_floats(toks, IDX_V)
        g = tokens_to_floats(toks, IDX_G)

        # Transform (only affected fields)
        p_new = Rzup_W.apply(p)
        v_new = Rzup_W.apply(v)
        g_new = Rzup_W.apply(g)
        q_new = (Rzup_W * Rot.from_quat(q)).as_quat()  # xyzw

        # Write back into *the same token list*, preserving all other tokens verbatim
        set_tokens_from_array(toks, IDX_P, p_new, is_quat=False)
        set_tokens_from_array(toks, IDX_Q, q_new, is_quat=True)
        set_tokens_from_array(toks, IDX_V, v_new, is_quat=False)
        set_tokens_from_array(toks, IDX_G, g_new, is_quat=False)

        # Note: toks[0] (time), bg, ba, extrinsic p & q remain their original strings.
        # Join with single spaces (original inter-token spacing is not preserved).
        out_lines.append(" ".join(toks))

    with open(args.outfile, "w") as f:
        for l in out_lines:
            f.write(l + "\n")

    # Sanity log
    g_mag = float(np.linalg.norm(g_world_10))
    g_new_check = Rzup_W.apply(g_world_10)
    print(f"[info] Used gravity at ~{args.gravity_from_start}s: {g_world_10} (|g|={g_mag:.6f})")
    print(f"[info] Rotated gravity check: {g_new_check} (should be ~[0, 0, -|g|])")
    if abs(g_new_check[0]) + abs(g_new_check[1]) > 1e-3 or g_new_check[2] > -0.5 * g_mag:
        print("[warn] Gravity after rotation is not close to [0,0,-|g|]. Check input conventions.", file=sys.stderr)

if __name__ == "__main__":
    main()
