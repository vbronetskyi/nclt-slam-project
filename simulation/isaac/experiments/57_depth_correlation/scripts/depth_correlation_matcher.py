#!/usr/bin/env python3
"""Exp 57 depth cross-correlation anchor matcher (grid template matching).

NOT ICP.  Deliberate choice of cross-correlation over ICP because:

  - VIO drift can reach 8 m; ICP's nearest-neighbour iteration would
    converge to a local minimum far from the true alignment.
  - Cones / tent add "new" cells not in the teach map; ICP would treat
    these as outliers and need tuning.  Our score only counts hits on
    teach-occupied cells and ignores everything else - cones naturally
    don't contribute.
  - Easier to debug: score surface is a small 2-D heatmap.

Runs in parallel with the ORB-based visual_landmark_matcher.  Both
publish `/anchor_correction`; tf_wall_clock_relay_v55 accepts whichever
is fresher.

Pipeline:
  1. Load teach occupancy grid from south_teach_map.pgm at startup.
  2. Subscribe /depth_points (2D point cloud in `camera_link`).
  3. At 1 Hz: project current frame's depth points to 2D world cells
     using the live VIO base pose, then search for the (dx, dy) offset
     (within ±SEARCH_WINDOW_M) that maximises how many current cells
     land on teach-occupied cells.
  4. If the best offset has at least MIN_HITS cells and the hit
     *fraction* exceeds `min_fraction_peak * 2nd_best_fraction`
     (significance test), publish `/anchor_correction`.

No ORB, no RANSAC.  Geometry-only: robust to cones/tent (new obstacles
just don't contribute hits - they dilute but don't mislead the peak).
"""
import argparse
import math
import os
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

# Camera -> base_link (same as visual_landmark_matcher)
BASE_TO_CAM_TRANSLATION = np.array([0.35, 0.0, 0.18])
BASE_TO_CAM_ROT = np.array([
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
    [1.0,  0.0,  0.0],
])

HEIGHT_MIN = 0.2
HEIGHT_MAX = 2.0          # match teach_run_depth_mapper (which used 0.2-2.0m)
MAX_RANGE = 15.0

SEARCH_WINDOW_M = 5.0     # ±5 m - accommodates larger drift near houses
SEARCH_STEP_M = 0.20
MIN_HITS = 5              # low - current scans can have only 40-60 total cells
MIN_FRACTION = 0.20       # up - require at least 20 % of scan to hit teach
MIN_PEAK_RATIO = 1.4      # up - require clear peak for confidence
CONSISTENCY_M = 5.0       # reject anchors >5 m from current VIO pose

TICK_HZ = 1.0


def quat_to_rot(qx, qy, qz, qw):
    R = np.eye(3)
    R[0, 0] = 1 - 2 * (qy*qy + qz*qz)
    R[0, 1] = 2 * (qx*qy - qz*qw)
    R[0, 2] = 2 * (qx*qz + qy*qw)
    R[1, 0] = 2 * (qx*qy + qz*qw)
    R[1, 1] = 1 - 2 * (qx*qx + qz*qz)
    R[1, 2] = 2 * (qy*qz - qx*qw)
    R[2, 0] = 2 * (qx*qz - qy*qw)
    R[2, 1] = 2 * (qy*qz + qx*qw)
    R[2, 2] = 1 - 2 * (qx*qx + qy*qy)
    return R


def load_teach_map(yaml_path, dilate_cells=1):
    """Load teach occupancy, optionally dilate to widen the "hit zone".

    The raw teach map from teach_run_depth_mapper is very sparse (~0.1 %
    occupied cells in forest scenes) because log-odds thresholding only
    promotes cells with strong evidence.  Correlation needs broader
    overlap to discriminate - we dilate by N cells (default 3 = 0.3 m),
    so a current-frame point within 30 cm of a teach-occupied cell
    counts as a hit.
    """
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    from PIL import Image
    img = np.array(Image.open(img_path))
    img = np.flipud(img)
    occupied = (img == 0)
    if dilate_cells > 0:
        # 3x3 square dilation repeated `dilate_cells` times
        from scipy.ndimage import binary_dilation
        occupied = binary_dilation(occupied, iterations=dilate_cells)
    return occupied, float(meta['origin'][0]), float(meta['origin'][1]), float(meta['resolution'])


def parse_pc2(msg):
    """Fast (N,3) float32 parse - identical to teach_run_depth_mapper."""
    if msg.point_step == 0 or msg.width == 0 or msg.height == 0:
        return np.zeros((0, 3), dtype=np.float32)
    offsets = {f.name: f.offset for f in msg.fields}
    if 'x' not in offsets:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    data = raw.reshape(n, msg.point_step)
    xs = np.frombuffer(data[:, offsets['x']:offsets['x']+4].tobytes(), dtype=np.float32)
    ys = np.frombuffer(data[:, offsets['y']:offsets['y']+4].tobytes(), dtype=np.float32)
    zs = np.frombuffer(data[:, offsets['z']:offsets['z']+4].tobytes(), dtype=np.float32)
    pts = np.stack([xs, ys, zs], axis=-1)
    return pts[np.isfinite(pts).all(axis=1)]


class DepthCorrelationMatcher(Node):
    def __init__(self, teach_yaml, log_csv):
        super().__init__('depth_correlation_matcher')
        self.teach_occ, self.origin_x, self.origin_y, self.res = load_teach_map(teach_yaml)
        self.H, self.W = self.teach_occ.shape
        self.get_logger().info(
            f'Loaded teach map {self.W}×{self.H} @ {self.res:.2f}m, '
            f'occupied cells: {int(self.teach_occ.sum())}')

        self.last_depth = None
        self.create_subscription(PointCloud2, '/depth_points',
                                 self._depth_cb, 10)
        self.anchor_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/anchor_correction', 10)

        self.timer = self.create_timer(1.0 / TICK_HZ, self._tick)
        self.n_attempts = 0
        self.n_published = 0
        self.log_csv = log_csv
        os.makedirs(os.path.dirname(log_csv), exist_ok=True)
        with open(log_csv, 'w') as f:
            f.write('ts,vio_x,vio_y,n_cells,best_dx,best_dy,best_hits,best_frac,peak_ratio,outcome\n')

        # Precompute search grid
        ns = int(SEARCH_WINDOW_M / SEARCH_STEP_M)
        self.search_offsets = []
        for i in range(-ns, ns + 1):
            for j in range(-ns, ns + 1):
                self.search_offsets.append((i * SEARCH_STEP_M, j * SEARCH_STEP_M))

    def _depth_cb(self, msg):
        self.last_depth = msg

    def _read_pose(self):
        try:
            with open('/tmp/isaac_pose.txt') as f:
                parts = f.readline().split()
            return tuple(float(p) for p in parts[:7])
        except Exception:
            return None

    def _log(self, ts, vio_xy, n_cells, dx, dy, hits, frac, peak_ratio, outcome):
        with open(self.log_csv, 'a') as f:
            f.write(f'{ts:.3f},{vio_xy[0]:.3f},{vio_xy[1]:.3f},{n_cells},'
                    f'{dx:.3f},{dy:.3f},{hits},{frac:.3f},{peak_ratio:.3f},'
                    f'{outcome}\n')

    def _tick(self):
        if self.last_depth is None:
            return
        base_pose = self._read_pose()
        if base_pose is None:
            return
        self.n_attempts += 1
        ts = time.time()
        vio_xy = (base_pose[0], base_pose[1])

        # Parse depth cloud
        pts_cam = parse_pc2(self.last_depth)
        if len(pts_cam) == 0:
            self._log(ts, vio_xy, 0, 0, 0, 0, 0, 0, 'no_depth_points')
            return

        # Transform to world via base + camera offset
        R_wb = quat_to_rot(*base_pose[3:7])
        R_wc = R_wb @ BASE_TO_CAM_ROT
        t_wc = np.array([base_pose[0], base_pose[1], base_pose[2]]) + \
               R_wb @ BASE_TO_CAM_TRANSLATION
        pts_w = (R_wc @ pts_cam.T).T + t_wc

        # Filter height + range
        z = pts_w[:, 2]
        h_mask = (z > HEIGHT_MIN) & (z < HEIGHT_MAX)
        dxy = np.hypot(pts_w[:, 0] - vio_xy[0], pts_w[:, 1] - vio_xy[1])
        r_mask = dxy < MAX_RANGE
        pts_w = pts_w[h_mask & r_mask]
        if len(pts_w) < 100:
            self._log(ts, vio_xy, len(pts_w), 0, 0, 0, 0, 0, 'too_few_points')
            return

        # Quantize to cells (unique)
        rows = ((pts_w[:, 1] - self.origin_y) / self.res).astype(np.int32)
        cols = ((pts_w[:, 0] - self.origin_x) / self.res).astype(np.int32)
        in_bounds = (rows >= 0) & (rows < self.H) & (cols >= 0) & (cols < self.W)
        rows = rows[in_bounds]; cols = cols[in_bounds]
        if len(rows) < 100:
            self._log(ts, vio_xy, len(rows), 0, 0, 0, 0, 0, 'out_of_bounds')
            return
        # unique cells
        rc = np.unique(np.stack([rows, cols], axis=-1), axis=0)
        n_cells = len(rc)

        # Search for best offset
        best_hits = 0
        best_dx = 0.0
        best_dy = 0.0
        scores = []
        step_cells = int(round(SEARCH_STEP_M / self.res))
        for dx, dy in self.search_offsets:
            dc = int(round(dx / self.res))
            dr = int(round(dy / self.res))
            shifted_r = rc[:, 0] + dr
            shifted_c = rc[:, 1] + dc
            valid = (shifted_r >= 0) & (shifted_r < self.H) & \
                    (shifted_c >= 0) & (shifted_c < self.W)
            if not valid.any():
                scores.append(0); continue
            hits = int(self.teach_occ[shifted_r[valid], shifted_c[valid]].sum())
            scores.append(hits)
            if hits > best_hits:
                best_hits = hits
                best_dx = dx
                best_dy = dy

        best_frac = best_hits / n_cells if n_cells > 0 else 0.0
        scores = np.array(scores)
        scores_sorted = np.sort(scores)[::-1]
        second_best = int(scores_sorted[1]) if len(scores_sorted) > 1 else 0
        peak_ratio = best_hits / max(1, second_best)

        if best_hits < MIN_HITS:
            self._log(ts, vio_xy, n_cells, best_dx, best_dy, best_hits,
                      best_frac, peak_ratio, 'low_hits')
            return
        if best_frac < MIN_FRACTION:
            self._log(ts, vio_xy, n_cells, best_dx, best_dy, best_hits,
                      best_frac, peak_ratio, 'low_fraction')
            return
        if peak_ratio < MIN_PEAK_RATIO:
            self._log(ts, vio_xy, n_cells, best_dx, best_dy, best_hits,
                      best_frac, peak_ratio, 'not_significant')
            return

        shift_m = math.hypot(best_dx, best_dy)
        if shift_m > CONSISTENCY_M:
            self._log(ts, vio_xy, n_cells, best_dx, best_dy, best_hits,
                      best_frac, peak_ratio, f'consistency_fail_{shift_m:.1f}')
            return

        # Anchor pose: robot is at vio + offset
        anchor_x = vio_xy[0] + best_dx
        anchor_y = vio_xy[1] + best_dy

        # Build covariance from peak-ratio (sharper peak -> smaller std)
        std = 0.10 + 0.15 / max(1.0, peak_ratio - 1.0)  # 0.1 - 0.25 m
        std = min(std, 0.25)
        cov = [0.0] * 36
        cov[0] = std * std
        cov[7] = std * std
        cov[14] = 0.25
        cov[21] = 0.05; cov[28] = 0.05; cov[35] = 0.05

        msg_out = PoseWithCovarianceStamped()
        msg_out.header.frame_id = 'map'
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.pose.pose.position.x = anchor_x
        msg_out.pose.pose.position.y = anchor_y
        msg_out.pose.pose.position.z = float(base_pose[2])
        msg_out.pose.pose.orientation.x = float(base_pose[3])
        msg_out.pose.pose.orientation.y = float(base_pose[4])
        msg_out.pose.pose.orientation.z = float(base_pose[5])
        msg_out.pose.pose.orientation.w = float(base_pose[6])
        msg_out.pose.covariance = cov
        self.anchor_pub.publish(msg_out)
        self.n_published += 1
        self._log(ts, vio_xy, n_cells, best_dx, best_dy, best_hits,
                  best_frac, peak_ratio,
                  f'published_std{std:.2f}_shift{shift_m:.2f}')

        if self.n_published % 10 == 0:
            self.get_logger().info(
                f'[ICP #{self.n_published}/{self.n_attempts}] '
                f'shift=({best_dx:+.2f},{best_dy:+.2f}) hits={best_hits}/{n_cells} '
                f'({100*best_frac:.0f}%) peak_ratio={peak_ratio:.2f} std={std:.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--teach-map', required=True)
    ap.add_argument('--out-csv', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = DepthCorrelationMatcher(args.teach_map, args.out_csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Final: {node.n_published} publishes / {node.n_attempts} attempts')
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
