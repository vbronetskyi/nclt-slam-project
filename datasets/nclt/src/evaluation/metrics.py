"""
ATE and RPE trajectory evaluation

Compares estimated vs ground truth trajectories in TUM format:
[timestamp, x, y, z, qx, qy, qz, qw]
"""
import numpy as np
from scipy.spatial.transform import Rotation


def compute_ate(traj_est, traj_gt):
    """ATE -- position L2 norm per frame, returns dict with mean/rmse/std/median/min/max/errors"""
    errs = np.linalg.norm(traj_est[:, 1:4] - traj_gt[:, 1:4], axis=1)
    return {
        'mean': errs.mean(),
        'rmse': np.sqrt((errs**2).mean()),
        'std': errs.std(),
        'median': np.median(errs),
        'min': errs.min(),
        'max': errs.max(),
        'errors': errs
    }


def compute_rpe(traj_est, traj_gt, delta=1):
    """RPE -- frame-to-frame relative transform error"""
    te, re = [], []
    for i in range(len(traj_est) - delta):
        def build_T(row):
            T = np.eye(4)
            T[:3, 3] = row[1:4]
            T[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
            return T

        Tgr = np.linalg.inv(build_T(traj_gt[i])) @ build_T(traj_gt[i + delta])
        Ter = np.linalg.inv(build_T(traj_est[i])) @ build_T(traj_est[i + delta])
        Tx = np.linalg.inv(Tgr) @ Ter
        te.append(np.linalg.norm(Tx[:3, 3]))
        re.append(np.degrees(
            np.arccos(np.clip((np.trace(Tx[:3, :3]) - 1) / 2, -1, 1))))

    te, re = np.array(te), np.array(re)
    return {
        'trans_rmse': np.sqrt((te**2).mean()),
        'trans_mean': te.mean(),
        'rot_rmse': np.sqrt((re**2).mean()),
        'rot_mean': re.mean(),
        'trans_errors': te,
        'rot_errors': re
    }


def sync_trajectories(est_traj, gt_all, tolerance=0.2):
    """sync est and gt by timestamp, returns (synced_est, synced_gt) same length"""
    gt_sync = []
    est_sync = []
    for i, ts in enumerate(est_traj[:, 0]):
        d = np.abs(gt_all[:, 0] - ts)
        mi = np.argmin(d)
        if d[mi] < tolerance:
            gt_sync.append(gt_all[mi])
            est_sync.append(est_traj[i])

    return np.array(est_sync), np.array(gt_sync)
