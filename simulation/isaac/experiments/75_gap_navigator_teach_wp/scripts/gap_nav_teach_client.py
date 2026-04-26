#!/usr/bin/env python3
"""Gap Navigator driven by teach waypoints
"""
import argparse, csv, math, sys, time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from gap_navigator import GapNavigator


def subsample(traj_path, spacing):
    pts = []
    with open(traj_path) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['gt_x']), float(row['gt_y'])))
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= spacing:
            out.append(p)
    return out


def robot_pose_from_tmp():
    try:
        with open('/tmp/isaac_pose.txt') as f:
            t = f.readline().strip().split()
            return float(t[0]), float(t[1])
    except Exception:
        return None


def skip_initial_reached(wps, robot_xy, tol=3.5):
    if robot_xy is None:
        return wps
    rx, ry = robot_xy
    i = 0
    while i < len(wps) and math.hypot(wps[i][0] - rx, wps[i][1] - ry) < tol:
        i += 1
    return wps[i:]


def depth_from_msg(msg):
    """Convert sensor_msgs/Image (depth) -> numpy HxW float32 meters.
    Supports 32FC1 and 16UC1 (assumed mm)."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding == '32FC1':
        arr = buf.view(np.float32).reshape(h, w).copy()
        arr[~np.isfinite(arr)] = 0.0
        return arr
    elif msg.encoding == '16UC1':
        arr = buf.view(np.uint16).reshape(h, w).astype(np.float32) / 1000.0
        return arr
    else:
        raise RuntimeError(f'unsupported depth encoding {msg.encoding}')


class GapNavTeachFollower(Node):
    TOLERANCE = 3.0
    CTRL_HZ = 10.0
    WP_TIMEOUT_S = 300.0     # give up on one WP after this
    FINAL_TAIL_N = 5         # last N WPs: never skip, 2x timeout
    STALL_DIST_M = 0.5       # must move this much per STALL_WINDOW_S or counts as stall
    STALL_WINDOW_S = 20.0
    MAX_RECOVERIES = 3       # per WP
    BACKUP_TIME_S = 3.0      # reverse for this long
    BACKUP_SPEED = -0.25
    ROTATE_TIME_S = 2.5      # rotate in place for this long (≈ 90° at 0.6 rad/s)
    ROTATE_SPEED = 0.6

    def __init__(self, wps):
        super().__init__('gap_nav_teach_follower')
        self.wps = wps
        self.reached = 0
        self.skipped = 0
        self.wp_idx = 0
        self.start = time.time()
        self.wp_started = time.time()
        self.nav = GapNavigator()
        self.last_depth = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self._on_depth, 5)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.stall_anchor_xy = None
        self.stall_anchor_t = time.time()
        # Recovery FSM: 'track', 'recover_backup', 'recover_rotate'
        self.state = 'track'
        self.state_started = time.time()
        self.recovery_count = 0
        self.rotate_direction = 1   # alternates each recovery

    def _on_depth(self, msg):
        try:
            self.last_depth = depth_from_msg(msg)
        except Exception as e:
            self.get_logger().warn(f'depth decode fail: {e}')

    def _robot_pose(self):
        try:
            tr = self.tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return None
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        q = tr.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    def _advance_wp(self, result):
        if result == 'reached':
            self.reached += 1
        else:
            self.skipped += 1
        self.wp_idx += 1
        self.wp_started = time.time()
        self.stall_anchor_xy = None
        self.stall_anchor_t = time.time()
        self.state = 'track'
        self.recovery_count = 0

    def _enter_recovery(self):
        """Start backup->rotate sequence."""
        self.recovery_count += 1
        self.state = 'recover_backup'
        self.state_started = time.time()
        self.rotate_direction *= -1
        self.get_logger().warn(
            f'  WP {self.wp_idx}/{len(self.wps)-1} RECOVERY '
            f'#{self.recovery_count} (backup->rotate dir={self.rotate_direction:+d})')

    def _stop(self):
        tw = Twist()
        self.cmd_pub.publish(tw)

    def run(self):
        rate = self.create_rate(self.CTRL_HZ)
        n = len(self.wps)
        self.get_logger().info(f'following {n} WPs with Gap Navigator')
        while rclpy.ok() and self.wp_idx < n:
            rclpy.spin_once(self, timeout_sec=1.0 / self.CTRL_HZ)
            pose = self._robot_pose()
            if pose is None or self.last_depth is None:
                continue
            rx, ry, ryaw = pose
            gx, gy = self.wps[self.wp_idx]
            d = math.hypot(gx - rx, gy - ry)
            is_final = self.wp_idx >= n - self.FINAL_TAIL_N
            if d < self.TOLERANCE:
                self.get_logger().info(
                    f'  WP {self.wp_idx}/{n-1} REACHED (d={d:.1f} m)')
                self._advance_wp('reached')
                continue

            # timeout
            budget = self.WP_TIMEOUT_S * (2.0 if is_final else 1.0)
            elapsed = time.time() - self.wp_started
            if elapsed > budget and not is_final:
                self.get_logger().warn(
                    f'  WP {self.wp_idx}/{n-1} TIMEOUT (d={d:.1f} m)')
                self._advance_wp('skipped')
                continue

            # Recovery FSM
            if self.state == 'recover_backup':
                tw = Twist()
                tw.linear.x = self.BACKUP_SPEED
                self.cmd_pub.publish(tw)
                if time.time() - self.state_started > self.BACKUP_TIME_S:
                    self.state = 'recover_rotate'
                    self.state_started = time.time()
                continue
            if self.state == 'recover_rotate':
                tw = Twist()
                tw.angular.z = self.ROTATE_SPEED * self.rotate_direction
                self.cmd_pub.publish(tw)
                if time.time() - self.state_started > self.ROTATE_TIME_S:
                    self.state = 'track'
                    # reset stall timer so we don't immediately re-trigger
                    self.stall_anchor_xy = (rx, ry)
                    self.stall_anchor_t = time.time()
                continue

            # stall detection (track state, not final tail)
            if self.stall_anchor_xy is None:
                self.stall_anchor_xy = (rx, ry)
                self.stall_anchor_t = time.time()
            if time.time() - self.stall_anchor_t > self.STALL_WINDOW_S:
                dx, dy = rx - self.stall_anchor_xy[0], ry - self.stall_anchor_xy[1]
                if math.hypot(dx, dy) < self.STALL_DIST_M:
                    # try recovery (unless maxed out or final tail)
                    if self.recovery_count < self.MAX_RECOVERIES and not is_final:
                        self._enter_recovery()
                        continue
                    if not is_final:
                        self.get_logger().warn(
                            f'  WP {self.wp_idx}/{n-1} STALL - {self.recovery_count} '
                            f'recoveries exhausted -> SKIP')
                        self._advance_wp('skipped')
                        continue
                # moved enough - reset anchor
                self.stall_anchor_xy = (rx, ry)
                self.stall_anchor_t = time.time()

            # heading error = desired_heading - robot_yaw
            desired = math.atan2(gy - ry, gx - rx)
            err = desired - ryaw
            err = math.atan2(math.sin(err), math.cos(err))

            lin, ang, mode, _dbg = self.nav.compute_cmd_vel(self.last_depth, err)
            tw = Twist()
            tw.linear.x = float(lin)
            tw.angular.z = float(ang)
            self.cmd_pub.publish(tw)

        self._stop()
        dur = int(time.time() - self.start)
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f'RESULT: reached {self.reached}/{n} '
            f'skipped {self.skipped} duration {dur}s')
        self.get_logger().info('=' * 50)
        return 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--spacing', type=float, default=4.0)
    args = ap.parse_args()

    wps = subsample(args.trajectory, args.spacing)
    print(f'Subsampled to {len(wps)} WPs at {args.spacing}m spacing')
    robot_xy = robot_pose_from_tmp()
    wps_after = skip_initial_reached(wps, robot_xy, tol=3.5)
    print(f'Skipped {len(wps) - len(wps_after)} leading WP(s) already within 3.5 m')
    wps = wps_after

    rclpy.init()
    node = GapNavTeachFollower(wps)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)
