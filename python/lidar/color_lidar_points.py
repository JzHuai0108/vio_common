# Given a seq of lidar point clouds and a seq of images, color the lidar points according to the image.
# The lidar point clouds have been undistorted by using a lidar odometry method, e.g., fastlio2.
# The images are subject to distortion and rolling shutter effect.
# We also require the reference state trajectory of the lidar sensor, the camera extrinsics relative to the lidar sensor
# and the IMU data and the IMU extrinsics. The IMU data are used to interpolate the pose at any time to deal with the rolling shutter effect.


# Inputs
# the lidar frames named with timestamps in a folder
# the image frames in a rosbag on img_topic
# the IMU data in a rosbag on imu_topic
# the reference states of TUM format in a txt file, each line is time, W_p_L, W_R_L, v_W, bg, ba, gravity_W
# the states correspond to the lidar point clouds exactly.


# Outputs
# the colored lidar point clouds

# Generally workflow

# brighten the images as the images may be too dark

# for each lidar point cloud with time t_lidar
#   find the image by timestamp with time diff less than tol = 0.1s, t_img,
#   for this purpose, we can keep an index of the previously used image, and start searching from there
#   also find the IMU data covering the time [a - tol, b + tol] where a = min(t_lidar, t_img) and b = max(t_lidar, t_img)
#   if t_lidar < t_img
    #   W_T_L(t_img) = propagate(state(t_lidar), imu_data, t_img)
#   else:
#       W_T_L(t_img) = propagate(state(t_lidar_prev), imu_data, t_img)
#   for each lidar point pt_lidar in the cloud,
    #   t_line = t_img
    #   for iter = 1:10
        #   W_T_L(t_line) = propagate(W_T_L(t_img), imu_data, t_line)
        #   pt_cam = C_T_L * inv(W_T_L(t_line)) * W_T_L(t_lidar) * pt_lidar
        #   pt_img = project(pt_cam, K, D)
        #   if pt_img is within the image and pt_img is close to its previous value tol = 0.1 pixel
        #       converged = true
        #       break
    #   if converged:
        #  if this pixel has not been assigned to another lidar point, or 
        #  if the pixel has been assigned to another lidar point pt_lidar_old, 
        #  but pt_lidar's depth in camera frame depth is smaller < depth_old or within 0.5 m to depth_old
        #       color = bilinear_interpolate(img, pt_img)
        #       pt_lidar.color = color
        #       if pt_lidar_old's depth is larger than pt_lidar's depth, and abs(depth_old - depth) > 0.5:
        #           pt_lidar_old.color = [255, 255, 255]
        #   else:
        #       pt_lidar.color = [0, 0, 0]


# Since the lidar FOV and the camera FOV are not the same, the above approach may miss lots of lidar points.
# To complement it, we may build a voxel grid out of all lidar points, and store the viewing images in each voxel, 
# i.e., each voxel stores the images that observes the voxel.
# then for every not colored lidar point, we find its voxel and use the image in that voxel to color the point.
# specifically, we retrieve a list of candidate images that may observe the lidar point and 
# pick the one with the viewing angle closest to the lidar point cloud's normal at the lidar point
# then, we project the lidar point into the image and color it with the bilinear interpolation.
# Obviously this projection will consider the rolling shutter effect with an iteration step as above.

import math
import os
import yaml
import numpy as np
import open3d as o3d
import cv2
import rosbag
from bisect import bisect_left, bisect_right
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from typing import Tuple


def on_mouse(event, x, y, flags, img):
    if event == cv2.EVENT_LBUTTONDOWN:
        b, g, r = img[y, x]
        print(f"Pixel clicked → col: {x}, row: {y}, BGR: ({b}, {g}, {r})")


def load_parameters(config_yaml):
    '''Load camera and IMU parameters from YAML.'''
    with open(config_yaml, 'r') as f:
        return yaml.safe_load(f)


def read_pcd_binary_with_intensity(path: str):
    """
    Load a binary PCD containing an 'intensity' field.
    The below code works fine by verifying with matlab code.
    Returns:
      - o3d.geometry.PointCloud with points
      - (N,) float array of intensities
    """
    with open(path, 'rb') as f:
        # ---- Parse header ----
        fields = sizes = types = counts = None
        width = height = points = None

        while True:
            line = f.readline().decode('utf-8').strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                sizes = list(map(int, line.split()[1:]))
            elif line.startswith('TYPE'):
                types = line.split()[1:]
            elif line.startswith('COUNT'):
                counts = list(map(int, line.split()[1:]))
            elif line.startswith('WIDTH'):
                width = int(line.split()[1])
            elif line.startswith('HEIGHT'):
                height = int(line.split()[1])
            elif line.startswith('POINTS'):
                points = int(line.split()[1])
            elif line.startswith('DATA') and 'binary' in line:
                # binary header ends here
                break

        if any(x is None for x in [fields, sizes, types, counts, points]):
            raise RuntimeError("Incomplete PCD header")

        # ---- Build NumPy dtype ----
        dtype = []
        for name, sz, typ, cnt in zip(fields, sizes, types, counts):
            # map PCD types to NumPy
            if typ == 'F':
                np_t = {4: 'f4', 8: 'f8'}[sz]
            elif typ == 'U':
                np_t = {1: 'u1', 2: 'u2', 4: 'u4'}[sz]
            elif typ == 'I':
                np_t = {1: 'i1', 2: 'i2', 4: 'i4'}[sz]
            else:
                raise RuntimeError(f"Unsupported PCD type: {typ}")
            if cnt == 1:
                dtype.append((name, np_t))
            else:
                dtype.append((name, np_t, (cnt,)))

        # ---- Read binary data ----
        data = np.fromfile(f, dtype=dtype, count=points)

    # ---- Extract fields ----
    pts = np.vstack((data['x'], data['y'], data['z'])).T
    intensity = data['intensity']

    # ---- Build Open3D PointCloud ----
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    return pcd, intensity


class State:
    def __init__(self, t, p, R_mat, v, bg, ba, g):
        self.t = t
        self.p = p # w_p_l
        self.R = R_mat # w_R_l
        self.v = v # w_v_l
        self.bg = bg
        self.ba = ba
        self.g = g # g_w

    def to_world(self, pt_l: np.ndarray) -> np.ndarray:
        """
        Transform a point from this LiDAR frame into world coordinates.
        pt_l: (3,) in LiDAR frame
        """
        return self.R.dot(pt_l) + self.p

    def to_lidar(self, pt_w: np.ndarray) -> np.ndarray:
        """
        Transform a world‐frame point into this LiDAR frame.
        pt_w: (3,) in world frame
        """
        return self.R.T.dot(pt_w - self.p)


def load_states(state_txt):
    '''
    Load reference states from a TUM-format text file.
    Each line: time, W_p_L (3), W_q_L (4 in x,y,z,w format), v_W (3), bg (3), ba (3), gravity_W (3)
    '''
    states = []
    with open(state_txt, 'r') as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            t = vals[0]
            p = np.array(vals[1:4])
            q = np.array(vals[4:8])  # quaternion [x, y, z, w]
            R_mat = R.from_quat(q).as_matrix()
            v = np.array(vals[8:11])
            bg = np.array(vals[11:14])
            ba = np.array(vals[14:17])
            g = np.array(vals[17:20])
            states.append(State(t, p, R_mat, v, bg, ba, g))
    return states


def bilinear_interpolate(img, uv):
    u,v=uv
    x0,y0=int(np.floor(u)),int(np.floor(v))
    x1,y1=min(x0+1,img.shape[1]-1),min(y0+1,img.shape[0]-1)
    a,b=u-x0,v-y0
    top=(1-a)*img[y0,x0]+a*img[y0,x1]
    bot=(1-a)*img[y1,x0]+a*img[y1,x1]
    return (1-b)*top+b*bot


class StateImage:
    def __init__(self, t, img, tr):
        '''
        t: adjusted timestamp (camera_time + td) in lidar clock
        img: BGR image ndarray
        tr: frame readout time (seconds between first and last row)
        '''
        self.t = t
        self.img = img
        self.tr = tr


class ImuPropagator:
    def __init__(self, imu_data, R_i_l, t_i_l):
        """
        imu_data: list of (timestamp, accel, gyro) tuples in IMU frame
        R_i_l   : rotation matrix (IMU -> LiDAR)
        t_i_l   : translation vector (IMU -> LiDAR)
        """
        self.imu_data = imu_data
        self.imu_times = [t for t,_,_ in self.imu_data]
        self.R_i_l = R_i_l
        self.t_i_l = np.array(t_i_l)
        # precompute the inverse transform (LiDAR -> IMU)
        self.R_l_i = R_i_l.T
        self.t_l_i = -self.R_l_i.dot(self.t_i_l)

    def propagate(self, state: State, t_target: float) -> State:
        """
        Convert pose at state.t → pose at t_target using IMU data,
        supporting both forward (t_target >= state.t) and backward integration.
        """
        # 1) LiDAR→IMU frame
        p_i   = state.R.dot(self.t_l_i) + state.p
        v_i   = state.v.copy()
        R_w_i = state.R.dot(self.R_l_i)
        bg, ba = state.bg, state.ba

        t0 = state.t
        t1 = t_target

        # find the IMU‐data indices that cover [min(t0,t1), max(t0,t1)]
        t_start, t_end = (t0, t1) if t1 >= t0 else (t1, t0)
        sid = bisect_left(self.imu_times, t_start)
        eid = bisect_right(self.imu_times, t_end)

        # build the list of indices to iterate
        if t1 >= t0:
            idxs       = range(sid, eid)
            last_index = eid - 1
        else:
            idxs       = range(eid - 1, sid - 1, -1)
            last_index = sid

        last_t = t0
        deltat = 0
        # if there’s absolutely no IMU data in the interval, just do one step
        if sid == eid:
            dt = t1 - t0
            # use the closest measurement for bias‐corrected accel/gyro
            a_raw, w_raw = self.imu_data[sid - 1][1:] if 0 < sid <= len(self.imu_data) else (ba, bg)
            w_i = w_raw - bg
            a_i = a_raw - ba
            a_w = R_w_i.dot(a_i) + state.g
            p_i += v_i * dt + 0.5 * a_w * dt**2
            v_i += a_w * dt
            R_w_i = R_w_i.dot(R.from_rotvec(w_i * dt).as_matrix())
            deltat = dt
        else:
            # integrate over each IMU sample
            for idx in idxs:
                t_i, a_raw, w_raw = self.imu_data[idx]
                # clamp the last interval so we end exactly at t1
                t_curr = t1 if idx == last_index else t_i
                dt     = t_curr - last_t

                # bias‐corrected measurements
                w_i = w_raw - bg
                a_i = a_raw - ba

                # world‐frame accel
                a_w = R_w_i.dot(a_i) + state.g

                # kinematic updates
                p_i += v_i * dt + 0.5 * a_w * dt**2
                v_i += a_w * dt

                # orientation update
                R_w_i = R_w_i.dot(R.from_rotvec(w_i * dt).as_matrix())

                last_t = t_curr
                deltat += dt

        expected = t_target - state.t
        assert math.isclose(deltat, expected, rel_tol=1e-6, abs_tol=1e-9), (
            f"deltat mismatch: got {deltat:.6f}, expected {expected:.6f}"
        )
        # 3) IMU→LiDAR frame
        p_l   = p_i + R_w_i.dot(self.t_i_l)
        R_w_l = R_w_i.dot(self.R_i_l)

        return State(t_target, p_l, R_w_l, v_i, bg, ba, state.g)


class IterativeProjector:
    def __init__(self, K, dist, wh, R_c_l, t_c_l, imu_prop,
                 max_iter=10, tol_px=0.04, depth_thresh=0.5):
        self.K = K
        self.dist = dist
        self.wh = wh
        self.R_c_l = R_c_l
        self.t_c_l = t_c_l
        self.imu_prop = imu_prop
        self.max_iter = max_iter
        self.tol_px = tol_px
        self.depth_thresh = depth_thresh

    def project_point(self, pt_lidar, state_lidar, state_img):
        """
        pt_lidar: point in the lidar frame at state_lidar.t
        """
        t_line = state_img.t
        pose_img = self.imu_prop.propagate(state_lidar, state_img.t)
        prev_uv = None
        start_uv = None
        for _ in range(self.max_iter):
            pose_line = self.imu_prop.propagate(pose_img, t_line)
            pt_w = state_lidar.R.dot(pt_lidar) + state_lidar.p
            pt_cam = self.R_c_l.dot(pose_line.R.T.dot(pt_w - pose_line.p)) + self.t_c_l
            if pt_cam[2] <= self.depth_thresh:
                return None, None
            x, y = pt_cam[0] / pt_cam[2], pt_cam[1] / pt_cam[2]
            r2 = x*x + y*y
            k1 = self.dist[0]
            k2 = self.dist[1]
            p1 = self.dist[2]
            p2 = self.dist[3]

            x_dist = x*(1 + k1*r2 + k2*r2*r2) + 2*p1*x*y + p2*(r2 + 2*x*x)
            y_dist = y*(1 + k1*r2 + k2*r2*r2) + p1*(r2 + 2*y*y) + 2*p2*x*y
            u = self.K[0,0]*x_dist + self.K[0,2]
            v = self.K[1,1]*y_dist + self.K[1,2]

            if not(0<=u<self.wh[0] and 0<=v<self.wh[1]):
                return None, None

            uv = np.array([u, v])
            if start_uv is None:
                start_uv = uv
            if prev_uv is not None and np.linalg.norm(uv-prev_uv) < self.tol_px:
                # print(f'rolling shutter shift {uv - start_uv}')
                return uv, pt_cam[2]
            prev_uv = uv
            # rolling shutter: use image height and tr
            t_line = state_img.t + (uv[1] / self.wh[1]) * state_img.tr
        return None, None


class VoxelGrid:
    def __init__(self, voxel_size, max_views=10):
        self.voxel_size = voxel_size
        self.max_views = max_views
        self.grid = {}  # voxel_idx -> list of (img_idx, score)

    def voxel_index(self, pt: np.ndarray) -> Tuple[int, ...]:
        """
        Compute voxel index by flooring the coordinate divided by voxel_size.
        This handles negative coords correctly:
          e.g. pt = [-0.1, 0.2], voxel_size=1 → idx = (-1, 0)
        """
        idx = np.floor(pt / self.voxel_size).astype(int)
        return tuple(idx)

    def add_observation(self, pt, img_idx, score):
        vid = self.voxel_index(pt)
        views = self.grid.setdefault(vid, [])
        if len(views) < self.max_views:
            views.append((img_idx, score))
        else:
            min_i, min_view = min(enumerate(views), key=lambda iv: iv[1][1])
            if score > min_view[1]:
                views[min_i] = (img_idx, score)

    def get_images(self, pt):
        vid = self.voxel_index(pt)
        return [img_idx for img_idx, _ in self.grid.get(vid, [])]


class Colorizer:
    def __init__(self, bag_path, pcd_dir, state_txt, config_yaml,
                 voxel_size=1.0, second_pass=False):
        self.bag_path = bag_path
        self.pcd_dir = pcd_dir
        self.states = load_states(state_txt)
        self.state_times = [s.t for s in self.states]
        params = load_parameters(config_yaml)

        # camera setup
        fx, fy, cx, cy = params['cam']['intrinsics']
        self.K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        self.dist = np.array(params['cam']['distortion'])
        self.wh = np.array(params['cam']['resolution'])
        self.img_td = params['cam']['td']
        self.img_tr = params['cam']['tr']
        R_c_l = R.from_quat(params['cam']['q_lidar_cam']).as_matrix()
        t_c_l = np.array(params['cam']['p_lidar_cam'])
        self.img_topic = params['cam']['topic']

        # imu setup
        R_i_l = np.array(params['imu']['R_imu_lidar']).reshape(3, 3)
        t_i_l = np.array(params['imu']['t_imu_lidar'])
        self.imu_topic = params['imu']['topic']

        self.imu_prop = ImuPropagator(self.load_imu(), R_i_l, t_i_l)
        self.projector = IterativeProjector(self.K, self.dist, self.wh, R_c_l, t_c_l, self.imu_prop)
        self.voxel_grid = VoxelGrid(voxel_size)
        self.second_pass = second_pass

        self.images = self.load_images(params['cam']['every_k'], maxk=params['cam']['maxk'], brighten=params['cam']['brighten'])
        self.image_times = [img.t for img in self.images]
        self.depth_maps = [dict() for _ in self.images]
        if second_pass:
            self._populate_voxel_grid(3)

    def load_imu(self):
        data=[]
        with rosbag.Bag(self.bag_path) as bag:
            for _, msg, _ in bag.read_messages(self.imu_topic):
                t=msg.header.stamp.to_sec()
                a=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
                w=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
                data.append((t, a, w))
        return data

    def load_images(self, every_k, maxk=1e9, brighten=False):
        imgs=[]
        with rosbag.Bag(self.bag_path) as bag:
            k = 0
            keep = 0
            first_frame = True
            bridge=CvBridge()
            if brighten:
                cv2.namedWindow('Orig vs Bright', cv2.WINDOW_NORMAL)
            else:
                cv2.namedWindow('Orig', cv2.WINDOW_NORMAL)
            for _, msg, _ in bag.read_messages(self.img_topic):
                if k % every_k != 0:
                    k += 1
                    continue
                t_cam=msg.header.stamp.to_sec()
                t=t_cam+self.img_td
                state = self.get_left_state(t, 0.1)
                if state is None:
                    k += 1
                    continue
                speed = np.linalg.norm(state.v)
                if speed < 0.05:
                    k += 1
                    continue

                img=bridge.imgmsg_to_cv2(msg,'bgr8')
                if brighten:
                    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
                    hsv[:,:,2]=cv2.equalizeHist(hsv[:,:,2])
                    bright=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
                    combo=cv2.hconcat([img,bright])
                    if first_frame:
                        h, w = combo.shape[:2]
                        cv2.resizeWindow('Orig vs Bright', w, h)
                        first_frame = False
                    cv2.imshow('Orig vs Bright',combo)
                else:
                    if first_frame:
                        h, w = img.shape[:2]
                        cv2.resizeWindow('Orig', w, h)
                        first_frame = False
                    # cv2.setMouseCallback('Orig', on_mouse, img)
                    cv2.imshow('Orig',img)
                    bright = img
                cv2.waitKey(1)
                imgs.append(StateImage(t,bright,self.img_tr))
                k += 1
                keep += 1
                if keep >= maxk:
                    break
        if brighten:
            cv2.destroyWindow('Orig vs Bright')
        else:
            cv2.destroyWindow('Orig')
        print(f'Loaded {len(imgs)} images')
        return imgs


    def _populate_voxel_grid(self, every_k):
        """
        Build world-frame normals on the dense aggregated cloud, downsample, then assign each
        downsampled center the normal of its nearest dense point, and record top-N views
        per voxel by normal alignment.
        """
        all_pts=[]
        print('Aggregating points to build the voxel grid')
        k = 0
        for fname in os.listdir(self.pcd_dir):
            if not fname.endswith('.pcd'): continue
            if k % every_k != 0:
                k += 1
                continue
            t_lidar=float(os.path.splitext(fname)[0])
            pcd=o3d.io.read_point_cloud(os.path.join(self.pcd_dir,fname))
            pts=np.asarray(pcd.points)
            state=self.get_state(t_lidar)
            pts_w=(state.R.dot(pts.T)).T+state.p # pts_w (N, 3)
            k += 1
            all_pts.append(pts_w)

        dense_pts=np.vstack(all_pts)
        dense_pcd=o3d.geometry.PointCloud()
        dense_pcd.points=o3d.utility.Vector3dVector(dense_pts)
        print('Estimating dense normals')
        dense_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4,max_nn=30))
        dense_normals=np.asarray(dense_pcd.normals) # dense_normals shape (N, 3)
        print('Downsampling whole point cloud')
        down_pcd=dense_pcd.voxel_down_sample(self.voxel_grid.voxel_size)
        centers=np.asarray(down_pcd.points)
        kdtree=o3d.geometry.KDTreeFlann(dense_pcd)
        center_normals=[]
        for c in centers:
            _,idx,_=kdtree.search_knn_vector_3d(c,1)
            center_normals.append(dense_normals[idx[0]])
        center_normals=np.vstack(center_normals)
        print('Filling voxels with views')
        for img_idx,img_state in enumerate(self.images):
            base_state=self.get_left_state(img_state.t, 0.07)
            if base_state is None:
                continue
            for center,normal in zip(centers,center_normals):
                pt_l=base_state.R.T.dot(center-base_state.p)
                uv,_=self.projector.project_point(pt_l,base_state,img_state)
                if uv is None: continue
                view_vec=(base_state.p-center)
                view_vec/=np.linalg.norm(view_vec)
                score=np.dot(view_vec,normal)
                self.voxel_grid.add_observation(center,img_idx,score)


    def check_lidar_projection(self, out_dir: str, every_k: int = 10):
        """
        For every_k consecutive PCD frames:
        1) Aggregate them into world-frame points (using their respective states)
        2) Project the aggregated points of the *first* frame onto its matching image
            coloring by the LiDAR intensity values
        3) Save the overlay image for visual inspection
        """
        os.makedirs(out_dir, exist_ok=True)

        # 1) collect and sort PCD filenames by timestamp
        fnames = [f for f in os.listdir(self.pcd_dir) if f.endswith('.pcd')]
        fnames.sort(key=lambda f: float(os.path.splitext(f)[0]))

        # 2) process in groups of every_k
        for i in range(0, len(fnames), every_k):
            group = fnames[i:i + every_k]
            if not group:
                continue

            # 3) aggregate world-frame points & intensities
            pts_w_list = []
            ints_list = []
            # the reference timestamp = first file in the group
            t_ref = float(os.path.splitext(group[0])[0])

            for fname in group:
                t_l = float(os.path.splitext(fname)[0])
                path = os.path.join(self.pcd_dir, fname)

                # load PCD + intensities
                pcd, intensities = read_pcd_binary_with_intensity(path)

                # get LiDAR state at that frame
                state = self.get_state(t_l)

                # transform points into world coords
                pts = np.asarray(pcd.points)            # (N,3) in LiDAR frame
                pts_w = (state.R.dot(pts.T)).T + state.p  # (N,3) in world
                pts_w_list.append(pts_w)
                ints_list.append(intensities)

            # skip if nothing loaded
            if not pts_w_list:
                continue

            all_pts_w = np.vstack(pts_w_list)       # (M,3)
            all_ints  = np.hstack(ints_list)        # (M,)

            # 4) transform aggregated world points into LiDAR frame at t_ref
            base_state = self.get_state(t_ref)
            # LiDAR frame = R^T * (world_pt - p)
            pts_l_ref = (base_state.R.T.dot((all_pts_w - base_state.p).T)).T  # (M,3)

            # 5) find the matching image
            img_idx, gap = self.find_image(t_ref, tol=0.1)
            if img_idx is None:
                print(f"[check] No image within tol for frame {t_ref}")
                continue
            img_state = self.images[img_idx]
            img = img_state.img.copy()  # BGR brightened image
            h, w = img.shape[:2]

            # 6) normalize intensities → [0,255] → JET colormap
            ints = all_ints.astype(np.float32)
            # guard against constant intensity
            if ints.max() > ints.min():
                norm = (ints - ints.min()) / (ints.max() - ints.min())
            else:
                norm = np.zeros_like(ints)
            norm_u8 = (norm * 255).astype(np.uint8)    # (M,)
            # applyColorMap expects a 2D array
            cmap_in = norm_u8[:, None]                # (M,1)
            colors = cv2.applyColorMap(cmap_in, cv2.COLORMAP_JET)  # (M,1,3)
            colors = colors.reshape(-1, 3)            # (M,3) BGR

            # 7) project & draw
            for pt_l, col in zip(pts_l_ref, colors):
                uv, _ = self.projector.project_point(pt_l, base_state, img_state)
                if uv is None:
                    continue
                u, v = uv
                ui, vi = int(round(u)), int(round(v))
                if not (0 <= ui < w and 0 <= vi < h):
                    continue
                # draw filled circle in BGR
                b, g, r = int(col[0]), int(col[1]), int(col[2])
                cv2.circle(img, (ui, vi), radius=2, color=(b, g, r), thickness=-1)

            # 8) save the overlay
            out_path = os.path.join(out_dir, f"{t_ref:.6f}_overlay.png")
            cv2.imwrite(out_path, img)
            print(f"[check] Saved overlay for {t_ref} → {out_path}")


    def colorize(self, out_dir):
        os.makedirs(out_dir,exist_ok=True)
        for fname in os.listdir(self.pcd_dir):
            if not fname.endswith('.pcd'): continue
            print(f'Colorizing point cloud {fname}')
            t_lidar=float(os.path.splitext(fname)[0])
            img_idx, gap = self.find_image(t_lidar, 0.04)
            if img_idx is None:
                print(f'Warn: unable to find right image for {t_lidar}, min gap {gap:.4f}, '
                      f'images start time {self.image_times[0]} end time {self.image_times[-1]}')
                continue
            img_state=self.images[img_idx]
            img=img_state.img
            pcd, intensities = read_pcd_binary_with_intensity(os.path.join(self.pcd_dir,fname))
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.7,max_nn=30))
            pts=np.asarray(pcd.points) # pts in lidar frame
            normals=np.asarray(pcd.normals)
            colors=np.zeros((len(pts),3))
            depth_map=self.depth_maps[img_idx]
            base_state=self.get_state(t_lidar)
            # first pass: color points with paired images
            k1 = 0
            for i,pt in enumerate(pts):
                uv,depth=self.projector.project_point(pt,base_state,img_state)
                if uv is None: continue
                u,v=uv
                ui,vi=int(round(u)),int(round(v))
                if not(0<=ui<self.wh[0] and 0<=vi<self.wh[1]):
                    continue
                key=(ui,vi)
                prev=depth_map.get(key)
                if prev is None or depth<prev[0]-self.projector.depth_thresh:
                    if prev is not None: colors[prev[1]]=np.ones(3)
                    colors[i]=bilinear_interpolate(img,uv)/255.0
                    depth_map[key]=(depth,i)
                    k1 += 1
            print(f'First pass colored {k1}/{len(pts)} points')

            if self.second_pass:
                # second pass: color points with distant images of overlapping FOV
                # The second pass does not check the ray occlusion, so it may wrongly color lidar points.
                second_indices = []
                k2 = 0
                for i, pt in enumerate(pts):
                    if np.any(colors[i]):
                        continue
                    best_score, best_uv, best_img = -1.0, None, None
                    pt_w = base_state.to_world(pt)
                    for vidx in self.voxel_grid.get_images(pt_w):
                        cimg = self.images[vidx]
                        state2 = self.get_left_state(cimg.t, tol=0.07)
                        if state2 is None:
                            print(f'Warn: unable to get left state for {cimg.t}')
                            continue
                        pt2 = state2.to_lidar(pt_w)
                        uv, depth = self.projector.project_point(pt2, state2, cimg)
                        if uv is None:
                            continue
                        view_vec = state2.p - pt_w
                        view_vec /= np.linalg.norm(view_vec)
                        normal_w = base_state.R.dot(normals[i])
                        score = float(np.dot(view_vec, normal_w))
                        if score > best_score:
                            best_score, best_uv, best_img = score, uv, cimg
                    if best_score > 0.5:
                        # print(f'best score {best_score:.5f}')
                        colors[i] = bilinear_interpolate(best_img.img, best_uv) / 255.0
                        second_indices.append(i)
                        k2 += 1
                print(f'Second pass colored {k2}/{len(pts)} points')
            # colors is in [B, G, R] order; convert to [R, G, B] for Open3D/CloudCompare
            colors_rgb = colors[:, [2, 1, 0]]

             # save *only* second‐pass points if requested
            if self.second_pass and second_indices:
                pcd2 = o3d.geometry.PointCloud()
                pcd2.points = o3d.utility.Vector3dVector(pts[second_indices])
                pcd2.colors = o3d.utility.Vector3dVector(colors_rgb[second_indices])
                out2 = os.path.join(out_dir, f"{os.path.splitext(fname)[0]}_second_only.ply")
                o3d.io.write_point_cloud(out2, pcd2)
                # print(f"  -> wrote second‐pass‐only PLY: {out2}")

            pcd.colors = o3d.utility.Vector3dVector(colors_rgb)
            out1=os.path.join(out_dir,f"{os.path.splitext(fname)[0]}_colored.ply")
            o3d.io.write_point_cloud(out1,pcd)
            # print(f"  -> wrote full colored PLY: {out1}")

    def get_state(self, t, tol=1e-6):
        # get a state of time very close to t within tol.
        i=bisect_left(self.state_times,t)
        if i<len(self.states) and abs(self.state_times[i]-t)<tol:
            return self.states[i]
        raise ValueError(f"State not found for time {t}")

    def get_left_state(self, t, tol):
        # get a state of time less than t and within tol.
        i = bisect_right(self.state_times,t)
        if i < len(self.states) and i > 0 and t - self.state_times[i-1] < tol:
            return self.states[i-1]
        return None

    def find_image(self, t_lidar, tol):
        i=bisect_left(self.image_times, t_lidar)
        if i<len(self.image_times):
            if self.image_times[i] - t_lidar < tol:
                return i, self.image_times[i] - t_lidar
            if i>0 and t_lidar - self.image_times[i-1] < tol:
                return i-1, t_lidar - self.image_times[i-1]
        return None, -1

    def _do_aggregate(self, outdir:str, suffix: str, out_name: str, remove_black_points: bool=False):
        world_pts = []
        world_cols = []
        for fname in sorted(os.listdir(outdir)):
            if not fname.endswith(suffix + '.ply'):
                continue
            path = os.path.join(outdir, fname)
            pcd  = o3d.io.read_point_cloud(path)
            pts  = np.asarray(pcd.points)
            cols = np.asarray(pcd.colors)

            # strip off the suffix to get the timestamp
            ts_str = fname[:-len(suffix + '.ply')]
            try:
                t = float(ts_str)
            except ValueError:
                continue

            state   = self.get_state(t)
            # world-frame transform
            pts_w = (state.R.dot(pts.T)).T + state.p

            world_pts.append(pts_w)
            world_cols.append(cols)

        if not world_pts:
            print(f"[aggregate] No '{suffix}' files found in {outdir}")
            return

        all_pts = np.vstack(world_pts)
        all_cols = np.vstack(world_cols)

        merged = o3d.geometry.PointCloud()
        merged.points = o3d.utility.Vector3dVector(all_pts)
        merged.colors = o3d.utility.Vector3dVector(all_cols)

        if remove_black_points:
            # build a boolean mask: True for any point that isn’t exactly [0,0,0]
            mask = np.any(all_cols != 0, axis=1)
            indices = np.nonzero(mask)[0]
            merged = merged.select_by_index(indices)

        count = np.asarray(merged.points).shape[0]
        out_path = os.path.join(outdir, out_name)
        o3d.io.write_point_cloud(out_path, merged)
        print(f"[aggregate] Wrote {out_name} ({count} pts)")


    def aggregate(self, outdir: str) -> None:
        """
        Aggregate both full-color and second-pass-only PLYs under `outdir`.
        Will produce:
          - aggregated_colored.ply
          - aggregated_second_only.ply
        """
        # aggregate the two kinds of clouds
        self._do_aggregate(outdir, suffix='_colored',         out_name='aggregated_colored.ply')
        self._do_aggregate(outdir, suffix='_second_only',     out_name='aggregated_second_only.ply')
        self._do_aggregate(outdir, suffix='_colored',         out_name='aggregated_white_only.ply', remove_black_points=True)


if __name__=='__main__':
    # example: python3 color_lidar_points.py /media/jhuai/ExtremeSSD/jhuai/livox_phone/s22plus_livox/20241205/2024_12_05_16_33_55/movie.bag \
    # /media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_livox/20241205/2024_12_05_16_33_55/PCD \
    # /media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_livox/20241205/2024_12_05_16_33_55/scan_states_odom.txt \
    # /home/jhuai/Documents/swift_vio_ws/src/vio_common/python/lidar/mid360_s22plus_config.yaml
    # /media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_livox/20241205/2024_12_05_16_33_55/color_pcd

    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('pcd_dir')
    parser.add_argument('state_txt')
    parser.add_argument('config_yaml')
    parser.add_argument('out_dir')
    parser.add_argument('--voxel_size',type=float,default=0.8)
    parser.add_argument('--second_pass', action='store_true', help='Enable the second-pass coloring (default: off)')

    args=parser.parse_args()
    cz=Colorizer(args.bag,args.pcd_dir,args.state_txt,args.config_yaml,
                 args.voxel_size, args.second_pass)
    # cz.check_lidar_projection(args.out_dir)
    cz.colorize(args.out_dir)
    cz.aggregate(args.out_dir)
