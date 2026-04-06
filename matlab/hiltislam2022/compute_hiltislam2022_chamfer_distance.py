#!/usr/bin/env python3
"""
compute_hiltislam2022_chamfer_distance.py

Evaluate LiDAR mapping accuracy for the HiltiSLAM 2022 dataset.

Pipeline (run step1_downsample.sh and precompute_transforms.m first):
  1. Load pre-downsampled result cloud (result_spatialsub_2cm.las).
  2. Apply precomputed initial alignment transform.
  3. Refine alignment with ICP (point-to-plane).
  4. Crop est to ref bounding box + 1 m buffer.
  5. Randomly downsample both clouds to the same point count.
  6. Compute Chamfer distance metrics (inspired by PIN_SLAM eval_mesh_utils.py).
  7. Save per-sequence results to chamfer_results.csv.

Sequences:
  exp04, exp05, exp06  ->  construction_site reference
  exp14, exp16, exp18  ->  sheldonian reference

Methods evaluated: balm2, balm3, base, nolc

Dependencies:
  pip install open3d laspy scipy numpy
"""

import csv
import os
import time

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

try:
    import laspy
    HAS_LASPY = True
except ImportError:
    HAS_LASPY = False
    print("[WARN] laspy not found; .las files cannot be loaded")

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BASE_DIR         = '/media/jhuai/T5EVO/jhuai/results/hiltislam2022_rss26'
METHODS          = ['balm2', 'balm3', 'base', 'nolc', 'lio']
# Per-method result filename (default: 'result_spatialsub_2cm.las')
METHOD_FILENAME  = {'lio': 'original_spatialsub_2cm.las'}
TRUNCATION_ACC   = 0.50   # outlier truncation for accuracy  direction (m) – PIN_SLAM default
TRUNCATION_COM   = 0.50   # outlier truncation for completion direction (m) – PIN_SLAM default
FSCORE_THRESHOLD = 0.05   # precision / recall / F-score threshold (m = 5 cm) – PIN_SLAM default
BBOX_BUFFER      = 1.0    # bounding-box buffer added on each axis (m)
VOXEL_SIZE       = 0.02   # voxel grid size for uniform downsampling of est cloud (m)
ICP_INLIER_DIST  = 1.0    # ICP max correspondence distance (m)
ICP_MAX_ITER     = 50
NORMAL_RADIUS    = 0.1    # normal estimation search radius (m)
NORMAL_MAX_NN    = 30     # normal estimation max neighbours

# {seq_dir, ref_key, pose_file_seq_id}
SEQ_CONFIG = [
    ('exp04', 'construction', 'exp4'),
    ('exp05', 'construction', 'exp5'),
    ('exp06', 'construction', 'exp6'),
    ('exp14', 'sheldonian',   'exp14'),
    ('exp16', 'sheldonian',   'exp16'),
    ('exp18', 'sheldonian',   'exp18'),
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_las(path: str):
    """Load a .las file and return (open3d PointCloud, elapsed_seconds)."""
    t0 = time.time()
    if HAS_LASPY:
        las = laspy.read(path)
        pts = np.vstack([las.x, las.y, las.z]).T.astype(np.float64)
    else:
        raise RuntimeError("laspy is required to read .las files")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd, time.time() - t0


def estimate_normals(pcd: o3d.geometry.PointCloud):
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=NORMAL_RADIUS, max_nn=NORMAL_MAX_NN))


def nn_distances(src_pts: np.ndarray, dst_pts: np.ndarray) -> np.ndarray:
    """Batch nearest-neighbour distances from each src point to dst (scipy cKDTree)."""
    tree = cKDTree(dst_pts)
    dists, _ = tree.query(src_pts, workers=-1)
    return dists


def voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Uniformly downsample a point array using a voxel grid (open3d)."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pcd_ds = pcd.voxel_down_sample(voxel_size)
    return np.asarray(pcd_ds.points)


def compute_metrics(pts_est: np.ndarray, pts_ref: np.ndarray,
                    truncation_acc: float, truncation_com: float,
                    fscore_thr: float) -> dict:
    """
    Chamfer L1/L2 metrics matching PIN_SLAM eval_mesh_utils.py exactly:
      - accuracy  (pred→GT):  outliers beyond truncation_acc are excluded
                              (nn_correspondance ignore_outlier=True)
      - completion (GT→pred): outliers beyond truncation_com are clamped
                              (nn_correspondance ignore_outlier=False)
      chamfer_l1 = 0.5 * (mean(d_p_inliers)      + mean(d_r_clamped))
      chamfer_l2 = sqrt(0.5 * (mean(d_p²_inliers) + mean(d_r²_clamped)))

    Args:
        pts_est       : (N, 3) estimated / predicted points
        pts_ref       : (M, 3) reference / ground-truth points
        truncation_acc: outlier cutoff for accuracy   direction (m)
        truncation_com: outlier cutoff for completion direction (m)
        fscore_thr    : threshold for precision & recall (m)

    Returns:
        dict with keys:
          mae_acc_cm, mae_com_cm          (mean absolute error in cm)
          chamfer_l1_cm, chamfer_l2_cm  (Chamfer L1 / L2 in cm)
          precision_pct, recall_pct, fscore_pct  (%)
    """
    d_est2ref = nn_distances(pts_est, pts_ref)   # accuracy direction
    d_ref2est = nn_distances(pts_ref, pts_est)   # completion direction

    # accuracy: exclude outliers (ignore_outlier=True)
    d_p = d_est2ref[d_est2ref < truncation_acc]
    # completion: clamp outliers to truncation_com (ignore_outlier=False)
    d_r = np.minimum(d_ref2est, truncation_com)

    mae_acc = float(np.mean(d_p)) * 100.0 if len(d_p) > 0 else float('nan')
    mae_com = float(np.mean(d_r)) * 100.0

    chamfer_l1 = 0.5 * (mae_acc + mae_com)

    var_p = float(np.mean(d_p ** 2)) if len(d_p) > 0 else float('nan')
    var_r = float(np.mean(d_r ** 2))
    chamfer_l2 = float(np.sqrt(0.5 * (var_p + var_r))) * 100.0 if not np.isnan(var_p) else float('nan')

    # precision / recall / F-score at fscore_thr
    precision = float(np.mean(d_est2ref < fscore_thr)) * 100.0
    recall    = float(np.mean(d_ref2est < fscore_thr)) * 100.0
    fscore    = (2.0 * precision * recall / (precision + recall)
                 if (precision + recall) > 0.0 else 0.0)

    return dict(mae_acc_cm=mae_acc, mae_com_cm=mae_com,
                chamfer_l1_cm=chamfer_l1, chamfer_l2_cm=chamfer_l2,
                precision_pct=precision, recall_pct=recall, fscore_pct=fscore)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ref_cache: dict[str, o3d.geometry.PointCloud] = {}

    col_headers = ['method', 'sequence', 'mae_acc_cm', 'mae_com_cm',
                   'chamfer_l1_cm', 'chamfer_l2_cm',
                   'precision_pct', 'recall_pct', 'fscore_pct']

    out_csv = os.path.join(BASE_DIR, 'chamfer_results.csv')
    csv_file = open(out_csv, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(col_headers)
    csv_file.flush()

    # Outer loop: sequence  (so each reference cloud is loaded once, then all
    # methods are evaluated against it before moving on)
    for seq, ref_key, pose_seq in SEQ_CONFIG:

        # -- load & cache reference cloud ------------------------------------
        if ref_key not in ref_cache:
            ref_las = os.path.join(BASE_DIR, 'ref', f'{ref_key}_spatialsub_2cm.las')
            print(f'\n  [ref] Loading {ref_las} ... ', end='', flush=True)
            ref_pcd, t_load = load_las(ref_las)
            print(f'done ({t_load:.1f} s, {len(ref_pcd.points):,} pts)')

            print(f'  [ref] Estimating normals ... ', end='', flush=True)
            t0 = time.time()
            estimate_normals(ref_pcd)
            print(f'done ({time.time() - t0:.1f} s)')

            ref_cache[ref_key] = ref_pcd

        ref_pcd = ref_cache[ref_key]
        pts_ref_full = np.asarray(ref_pcd.points)

        # -- inner loop: method ----------------------------------------------
        for method in METHODS:
            print(f'\n=== [{method}/{seq}] ===')

            filename   = METHOD_FILENAME.get(method, 'result_spatialsub_2cm.las')
            result_las = os.path.join(BASE_DIR, method, seq, filename)
            if not os.path.isfile(result_las):
                print(f'  [SKIP] Missing: {result_las}')
                continue

            # -- load est cloud ----------------------------------------------
            print(f'[{method}/{seq}] Loading est cloud ... ', end='', flush=True)
            est_pcd, t_load = load_las(result_las)
            print(f'done ({t_load:.1f} s, {len(est_pcd.points):,} pts)')

            # -- initial alignment -------------------------------------------
            pose_file = os.path.join(
                BASE_DIR, 'initial_poses',
                f'{ref_key}_ref_T_{pose_seq}_est_precomputed.txt')
            if not os.path.isfile(pose_file):
                raise FileNotFoundError(
                    f'Missing precomputed transform: {pose_file}\n'
                    'Run precompute_transforms.m first.')

            T_init = np.loadtxt(pose_file)
            R = T_init[:3, :3]
            U, _, Vt = np.linalg.svd(R)
            R = U @ Vt
            if np.linalg.det(R) < 0:
                U[:, 2] = -U[:, 2]
                R = U @ Vt
            T4 = np.eye(4)
            T4[:3, :3] = R
            T4[:3, 3]  = T_init[:3, 3]

            print(f'[{method}/{seq}] Applying initial alignment ... ', end='', flush=True)
            t0 = time.time()
            est_pcd.transform(T4)
            print(f'done ({time.time() - t0:.1f} s)')

            # -- ICP refinement (point-to-plane) -----------------------------
            print(f'[{method}/{seq}] Running ICP (point-to-plane, '
                  f'max_iter={ICP_MAX_ITER}) ... ', end='', flush=True)
            t0 = time.time()
            estimate_normals(est_pcd)
            icp_result = o3d.pipelines.registration.registration_icp(
                est_pcd, ref_pcd,
                max_correspondence_distance=ICP_INLIER_DIST,
                estimation_method=(
                    o3d.pipelines.registration
                    .TransformationEstimationPointToPlane()),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=ICP_MAX_ITER,
                    relative_fitness=1e-6,
                    relative_rmse=1e-6))
            est_pcd.transform(icp_result.transformation)
            print(f'done ({time.time() - t0:.1f} s, '
                  f'fitness={icp_result.fitness:.4f}, '
                  f'rmse={icp_result.inlier_rmse:.4f} m)')

            # -- crop est to ref bbox + buffer --------------------------------
            pts_est = np.asarray(est_pcd.points)
            ref_min = pts_ref_full.min(axis=0) - BBOX_BUFFER
            ref_max = pts_ref_full.max(axis=0) + BBOX_BUFFER
            in_box  = np.all((pts_est >= ref_min) & (pts_est <= ref_max), axis=1)
            pts_est_cropped = pts_est[in_box]
            print(f'[{method}/{seq}] Cropped est: {len(pts_est):,} -> '
                  f'{len(pts_est_cropped):,} pts '
                  f'(ref bbox + {BBOX_BUFFER:.0f} m buffer)')

            # -- uniform voxel downsample est cloud --------------------------
            pts_est_ds = voxel_downsample(pts_est_cropped, VOXEL_SIZE)
            print(f'[{method}/{seq}] Voxel downsample est '
                  f'(voxel={VOXEL_SIZE * 100:.0f} cm): '
                  f'{len(pts_est_cropped):,} -> {len(pts_est_ds):,} pts  '
                  f'ref: {len(pts_ref_full):,} pts')

            # -- Chamfer distance metrics ------------------------------------
            print(f'[{method}/{seq}] Computing Chamfer distance '
                  f'(trunc_acc={TRUNCATION_ACC * 100:.0f} cm, '
                  f'trunc_com={TRUNCATION_COM * 100:.0f} cm, '
                  f'fscore_thr={FSCORE_THRESHOLD * 100:.0f} cm) ... ',
                  end='', flush=True)
            t0 = time.time()
            m = compute_metrics(pts_est_ds, pts_ref_full,
                                TRUNCATION_ACC, TRUNCATION_COM, FSCORE_THRESHOLD)
            print(f'done ({time.time() - t0:.1f} s)')

            print(f'[{method}/{seq}] '
                  f'MAE_acc={m["mae_acc_cm"]:.2f} cm  '
                  f'MAE_com={m["mae_com_cm"]:.2f} cm  '
                  f'chamfer_L1={m["chamfer_l1_cm"]:.2f} cm  '
                  f'chamfer_L2={m["chamfer_l2_cm"]:.2f} cm  '
                  f'precision={m["precision_pct"]:.1f}%  '
                  f'recall={m["recall_pct"]:.1f}%  '
                  f'fscore={m["fscore_pct"]:.1f}%')

            csv_writer.writerow([method, seq] + [f'{m[k]:.4f}' for k in col_headers[2:]])
            csv_file.flush()
            print(f'[{method}/{seq}] Row written to CSV.')

    csv_file.close()
    print(f'\nResults saved to: {out_csv}')


if __name__ == '__main__':
    main()
