#!/usr/bin/env python3
"""
Check consistency between two KITTI odometry Velodyne scans.

Default workflow:
  - Load scans 82 and 1553 from KITTI sequence 00 .bin files.
  - Load KITTI odometry poses from poses/00.txt.
  - Load calib.txt:Tr, interpreted by default as cam0_from_velodyne.
  - Transform scan 1553 into scan 82's Velodyne frame.
  - Print nearest-neighbor consistency statistics.
  - Save the full transformed source scan and target scan as binary XYZI PCDs
    for visualization.

KITTI odometry pose convention:
  Each line is interpreted as T_cam0_from_cam_i, i.e. a point in camera frame i
  is transformed into camera frame 0 by the 3x4 matrix on line i. This is also
  equivalent to "world_from_cam" if frame 0 is treated as the world frame.
"""

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree


DEFAULT_VELODYNE_DIR = Path(
    "/media/jhuai/BackupPlus1/jhuai/data/KITTI/data_odometry_velodyne/"
    "sequences/00/velodyne"
)
DEFAULT_POSES_PATH = Path(
    "/media/jhuai/BackupPlus1/jhuai/data/KITTI/data_odometry_poses/"
    "dataset/poses/00.txt"
)
DEFAULT_CALIB_PATH = Path(
    "/media/jhuai/BackupPlus1/jhuai/data/KITTI/data_odometry_calib/"
    "dataset/sequences/00/calib.txt"
)
DEFAULT_OUTPUT_PCD = Path("/tmp/kitti_seq00_1553_in_82_velodyne.pcd")
DEFAULT_TARGET_OUTPUT_PCD = Path("/tmp/kitti_seq00_82_velodyne.pcd")


def make_transform(mat3x4: np.ndarray) -> np.ndarray:
    """Convert a 3x4 pose/calib matrix into a 4x4 homogeneous transform."""
    if mat3x4.shape != (3, 4):
        raise ValueError(f"Expected a 3x4 matrix, got {mat3x4.shape}")

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :4] = mat3x4.astype(np.float64)
    return transform


def load_kitti_bin(bin_path: Path) -> np.ndarray:
    """Load a KITTI Velodyne .bin as an Nx4 float32 array: x, y, z, intensity."""
    if not bin_path.exists():
        raise FileNotFoundError(bin_path)

    points = np.fromfile(str(bin_path), dtype=np.float32)
    if points.size % 4 != 0:
        raise ValueError(f"{bin_path} does not contain a multiple of 4 float32 values")

    return points.reshape((-1, 4))


def load_poses(poses_path: Path) -> List[np.ndarray]:
    """Load KITTI poses as 4x4 transforms, one per line."""
    if not poses_path.exists():
        raise FileNotFoundError(poses_path)

    poses: List[np.ndarray] = []
    with poses_path.open("r") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue

            values = np.fromstring(line, sep=" ", dtype=np.float64)
            if values.size != 12:
                raise ValueError(
                    f"{poses_path}:{line_no} expected 12 values, got {values.size}"
                )
            poses.append(make_transform(values.reshape(3, 4)))

    return poses


def load_calib(calib_path: Path) -> Dict[str, np.ndarray]:
    """Load KITTI odometry calibration entries that contain 12 matrix values."""
    if not calib_path.exists():
        raise FileNotFoundError(calib_path)

    calib: Dict[str, np.ndarray] = {}
    with calib_path.open("r") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue

            if ":" not in line:
                raise ValueError(f"{calib_path}:{line_no} missing ':' separator")

            key, value_text = line.split(":", 1)
            values = np.fromstring(value_text, sep=" ", dtype=np.float64)
            if values.size == 12:
                calib[key.strip()] = make_transform(values.reshape(3, 4))

    return calib


def get_calib_transform(calib: Dict[str, np.ndarray], key: str) -> np.ndarray:
    """Return a calibration transform with a few KITTI naming fallbacks."""
    candidates = [key]
    if key == "Tr":
        candidates.extend(["Tr_velo_to_cam", "Tr_velo_cam"])

    for candidate in candidates:
        if candidate in calib:
            return calib[candidate]

    available = ", ".join(sorted(calib.keys()))
    raise KeyError(f"Calibration key '{key}' not found. Available keys: {available}")


def transform_points(points_xyz: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an Nx3 point array."""
    return points_xyz @ transform[:3, :3].T + transform[:3, 3]


def finite_xyz_intensity(points_xyzi: np.ndarray) -> np.ndarray:
    """Drop points with non-finite xyz/intensity values."""
    return points_xyzi[np.isfinite(points_xyzi).all(axis=1)]


def range_filter(points_xyzi: np.ndarray, max_range: float) -> np.ndarray:
    """Keep points whose Euclidean xyz range is <= max_range."""
    if max_range <= 0:
        raise ValueError("--max-range must be positive when provided")

    ranges = np.linalg.norm(points_xyzi[:, :3], axis=1)
    return points_xyzi[ranges <= max_range]


def voxel_downsample_xyz(points_xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """Simple deterministic voxel downsampling for metric computation."""
    if voxel_size <= 0.0 or points_xyz.shape[0] == 0:
        return points_xyz

    voxel_index = np.floor(points_xyz / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(voxel_index, axis=0, return_index=True)
    return points_xyz[np.sort(unique_indices)]


def relative_camera_transform(
    poses: List[np.ndarray],
    source_frame: int,
    target_frame: int,
    pose_convention: str,
) -> np.ndarray:
    """
    Return T_target_cam_from_source_cam.

    kitti_cam_to_first/world_from_cam:
      pose[i] maps cam_i -> common frame, so target<-source is inv(pose[target]) @ pose[source].

    cam_from_world:
      pose[i] maps common frame -> cam_i, so target<-source is pose[target] @ inv(pose[source]).
    """
    max_frame = max(source_frame, target_frame)
    if max_frame >= len(poses):
        raise IndexError(f"Need pose index {max_frame}, but only loaded {len(poses)} poses")

    pose_source = poses[source_frame]
    pose_target = poses[target_frame]

    if pose_convention in ("kitti_cam_to_first", "world_from_cam"):
        return np.linalg.inv(pose_target) @ pose_source
    if pose_convention == "cam_from_world":
        return pose_target @ np.linalg.inv(pose_source)

    raise ValueError(f"Unsupported pose convention: {pose_convention}")


def relative_lidar_transform(
    poses: List[np.ndarray],
    calib_tr: np.ndarray,
    source_frame: int,
    target_frame: int,
    pose_convention: str,
    tr_convention: str,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return T_target_lidar_from_source_lidar and the intermediate camera transform."""
    target_cam_from_source_cam = relative_camera_transform(
        poses, source_frame, target_frame, pose_convention
    )

    if tr_convention == "cam_from_lidar":
        cam_from_lidar = calib_tr
        target_lidar_from_source_lidar = (
            np.linalg.inv(cam_from_lidar)
            @ target_cam_from_source_cam
            @ cam_from_lidar
        )
    elif tr_convention == "lidar_from_cam":
        lidar_from_cam = calib_tr
        target_lidar_from_source_lidar = (
            lidar_from_cam
            @ target_cam_from_source_cam
            @ np.linalg.inv(lidar_from_cam)
        )
    else:
        raise ValueError(f"Unsupported Tr convention: {tr_convention}")

    return target_lidar_from_source_lidar, target_cam_from_source_cam


def write_binary_pcd_xyzi(pcd_path: Path, points_xyzi: np.ndarray) -> None:
    """Write an XYZI cloud as a PCL-compatible binary PCD file."""
    pcd_path.parent.mkdir(parents=True, exist_ok=True)

    cloud = np.empty(
        points_xyzi.shape[0],
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
        ],
    )
    cloud["x"] = points_xyzi[:, 0].astype(np.float32, copy=False)
    cloud["y"] = points_xyzi[:, 1].astype(np.float32, copy=False)
    cloud["z"] = points_xyzi[:, 2].astype(np.float32, copy=False)
    cloud["intensity"] = points_xyzi[:, 3].astype(np.float32, copy=False)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {cloud.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {cloud.shape[0]}\n"
        "DATA binary\n"
    )

    with pcd_path.open("wb") as f:
        f.write(header.encode("ascii"))
        cloud.tofile(f)


def nearest_neighbor_stats(
    query_xyz: np.ndarray,
    reference_xyz: np.ndarray,
    thresholds: Iterable[float],
) -> Dict[str, float]:
    """Compute nearest-neighbor distance statistics from query to reference."""
    if query_xyz.shape[0] == 0:
        raise ValueError("Query point set is empty")
    if reference_xyz.shape[0] == 0:
        raise ValueError("Reference point set is empty")

    tree = cKDTree(reference_xyz)
    try:
        distances, _ = tree.query(query_xyz, k=1, workers=-1)
    except TypeError:
        distances, _ = tree.query(query_xyz, k=1)

    stats = {
        "count": float(distances.size),
        "mean": float(np.mean(distances)),
        "std": float(np.std(distances)),
        "median": float(np.median(distances)),
        "p90": float(np.percentile(distances, 90)),
        "p95": float(np.percentile(distances, 95)),
        "p99": float(np.percentile(distances, 99)),
        "max": float(np.max(distances)),
    }
    for threshold in thresholds:
        stats[f"ratio_lt_{threshold:g}m"] = float(np.mean(distances < threshold))

    return stats


def print_matrix(name: str, matrix: np.ndarray) -> None:
    print(f"\n{name}:")
    for row in matrix:
        print("  " + " ".join(f"{v: .9f}" for v in row))


def print_stats(name: str, stats: Dict[str, float]) -> None:
    print(f"\n{name}:")
    print(f"  points: {int(stats['count'])}")
    for key in ("mean", "std", "median", "p90", "p95", "p99", "max"):
        print(f"  {key}: {stats[key]:.4f} m")

    for key in sorted(k for k in stats if k.startswith("ratio_lt_")):
        threshold = key[len("ratio_lt_") :]
        print(f"  {threshold}: {100.0 * stats[key]:.2f}%")


def optional_path(value: str) -> Optional[Path]:
    """Argparse helper: use an empty string to disable an output path."""
    if value == "":
        return None
    return Path(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Transform one KITTI Velodyne scan into another frame and check point consistency."
    )
    parser.add_argument("--velodyne-dir", type=Path, default=DEFAULT_VELODYNE_DIR)
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES_PATH)
    parser.add_argument("--calib", type=Path, default=DEFAULT_CALIB_PATH)
    parser.add_argument("--source-frame", type=int, default=1553)
    parser.add_argument("--target-frame", type=int, default=82)
    parser.add_argument("--calib-key", type=str, default="Tr")
    parser.add_argument(
        "--pose-convention",
        choices=("kitti_cam_to_first", "world_from_cam", "cam_from_world"),
        default="kitti_cam_to_first",
        help="Convention used by the 3x4 pose rows.",
    )
    parser.add_argument(
        "--tr-convention",
        choices=("cam_from_lidar", "lidar_from_cam"),
        default="cam_from_lidar",
        help="Convention used by calib.txt:Tr.",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=80.0,
        help="Keep points within this range in their comparison frame. Use <=0 to disable.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.2,
        help="Voxel size for metric computation. Use <=0 to disable downsampling.",
    )
    parser.add_argument(
        "--thresholds",
        type=float,
        nargs="+",
        default=[0.1, 0.2, 0.5, 1.0],
        help="Distance thresholds for nearest-neighbor inlier ratios.",
    )
    parser.add_argument(
        "--output-pcd",
        type=optional_path,
        default=DEFAULT_OUTPUT_PCD,
        help=(
            "Output XYZI PCD for the full transformed source scan. "
            "Use an empty string to disable."
        ),
    )
    parser.add_argument(
        "--target-output-pcd",
        type=optional_path,
        default=DEFAULT_TARGET_OUTPUT_PCD,
        help="Output XYZI PCD for the target scan. Use an empty string to disable.",
    )
    parser.add_argument(
        "--no-symmetric-check",
        action="store_true",
        help="Only compute transformed source -> target nearest-neighbor stats.",
    )
    parser.add_argument(
        "--print-matrices",
        action="store_true",
        help="Print the relative camera and lidar transforms.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    source_bin = args.velodyne_dir / f"{args.source_frame:06d}.bin"
    target_bin = args.velodyne_dir / f"{args.target_frame:06d}.bin"

    source_points = finite_xyz_intensity(load_kitti_bin(source_bin))
    target_points = finite_xyz_intensity(load_kitti_bin(target_bin))
    poses = load_poses(args.poses)
    calib = load_calib(args.calib)
    calib_tr = get_calib_transform(calib, args.calib_key)

    target_lidar_from_source_lidar, target_cam_from_source_cam = relative_lidar_transform(
        poses=poses,
        calib_tr=calib_tr,
        source_frame=args.source_frame,
        target_frame=args.target_frame,
        pose_convention=args.pose_convention,
        tr_convention=args.tr_convention,
    )

    transformed_source_xyz = transform_points(
        source_points[:, :3], target_lidar_from_source_lidar
    )
    transformed_source_points = np.column_stack(
        (transformed_source_xyz, source_points[:, 3])
    ).astype(np.float32)

    if args.output_pcd is not None:
        write_binary_pcd_xyzi(args.output_pcd, transformed_source_points)
    if args.target_output_pcd is not None:
        write_binary_pcd_xyzi(args.target_output_pcd, target_points)

    target_for_metrics = target_points
    source_for_metrics = transformed_source_points
    if args.max_range > 0.0:
        target_for_metrics = range_filter(target_for_metrics, args.max_range)
        source_for_metrics = range_filter(source_for_metrics, args.max_range)

    target_metric_xyz = voxel_downsample_xyz(target_for_metrics[:, :3], args.voxel_size)
    source_metric_xyz = voxel_downsample_xyz(source_for_metrics[:, :3], args.voxel_size)

    source_to_target = nearest_neighbor_stats(
        source_metric_xyz, target_metric_xyz, args.thresholds
    )

    print(f"Loaded source scan {args.source_frame}: {source_points.shape[0]} points")
    print(f"Loaded target scan {args.target_frame}: {target_points.shape[0]} points")
    print(
        f"Metric clouds after range/downsample: source={source_metric_xyz.shape[0]}, "
        f"target={target_metric_xyz.shape[0]}"
    )
    print(f"Pose convention: {args.pose_convention}")
    print(f"Tr convention: {args.tr_convention}")
    if args.output_pcd is not None:
        print(f"Saved transformed source scan: {args.output_pcd}")
    if args.target_output_pcd is not None:
        print(f"Saved target scan: {args.target_output_pcd}")

    if args.print_matrices:
        print_matrix(
            f"T_cam{args.target_frame}_from_cam{args.source_frame}",
            target_cam_from_source_cam,
        )
        print_matrix(
            f"T_lidar{args.target_frame}_from_lidar{args.source_frame}",
            target_lidar_from_source_lidar,
        )

    print_stats(
        f"NN transformed scan {args.source_frame} -> scan {args.target_frame}",
        source_to_target,
    )

    if not args.no_symmetric_check:
        target_to_source = nearest_neighbor_stats(
            target_metric_xyz, source_metric_xyz, args.thresholds
        )
        print_stats(
            f"NN scan {args.target_frame} -> transformed scan {args.source_frame}",
            target_to_source,
        )
        chamfer_mean = 0.5 * (source_to_target["mean"] + target_to_source["mean"])
        print(f"\nSymmetric mean nearest-neighbor distance: {chamfer_mean:.4f} m")


if __name__ == "__main__":
    main()
