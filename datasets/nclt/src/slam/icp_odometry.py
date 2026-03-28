"""
Custom ICP-based LiDAR odometry

Point-to-plane ICP via Open3D for frame-to-frame pose estimation
"""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


class CustomICP:
    """Frame-to-frame ICP odometry via Open3D point-to-plane.
    Voxel downsample + normal estimation + ICP against previous scan.
    """

    def __init__(self, voxel_size=0.3, threshold=1.5, max_iter=100):
        self.voxel_size = voxel_size
        self.threshold = threshold
        self.max_iter = max_iter
        self.poses = []
        self.timestamps = []
        self.current_pose = np.eye(4)
        self.prev_pcd = None

    def register_scan(self, points, timestamp, init_transform=None):
        """register new scan, update trajectory, returns current 4x4 global pose"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

        if self.prev_pcd is None:
            self.poses.append(self.current_pose.copy())
            self.timestamps.append(timestamp)
            self.prev_pcd = pcd
            return self.current_pose

        if init_transform is None:
            init_transform = np.eye(4)

        reg = o3d.pipelines.registration.registration_icp(
            pcd, self.prev_pcd, self.threshold, init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.max_iter,
                relative_fitness=1e-6, relative_rmse=1e-6))

        self.current_pose = self.current_pose @ reg.transformation
        self.poses.append(self.current_pose.copy())
        self.timestamps.append(timestamp)
        self.prev_pcd = pcd

        return self.current_pose

    def get_trajectory(self):
        """Nx8 array [timestamp_s, x, y, z, qx, qy, qz, qw]"""
        traj = []
        for timestamp, pose in zip(self.timestamps, self.poses):
            pos = pose[:3, 3]
            quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
            traj.append([timestamp / 1e6, pos[0], pos[1], pos[2],
                         quat[0], quat[1], quat[2], quat[3]])
        return np.array(traj)
