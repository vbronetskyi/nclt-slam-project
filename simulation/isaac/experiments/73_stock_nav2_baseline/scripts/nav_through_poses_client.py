#!/usr/bin/env python3
"""Stock-Nav2 baseline client

Reads the teach trajectory CSV (same 4 m subsampling our send_goals_hybrid
does) and sends the full WP list to /navigate_through_poses as a single
action goal. Logs per-feedback progress (number_of_poses_remaining,
current_pose, navigation_duration) until the server returns, then prints
a RESULT line compatible with the orchestrator's wait-loop.
"""
import argparse, csv, math, time, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


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


def pose_from_xy(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    return p


class NavClient(Node):
    def __init__(self, wps):
        super().__init__('nav_through_poses_client')
        self.wps = wps
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.done = False
        self.last_remaining = None
        self.start = time.time()

    def run(self):
        self.get_logger().info(f'Waiting for /navigate_through_poses server...')
        if not self.client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('NavigateThroughPoses server did not come up')
            return 1
        poses = []
        for i, (x, y) in enumerate(self.wps):
            yaw = 0.0
            if i + 1 < len(self.wps):
                nx, ny = self.wps[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            poses.append(pose_from_xy(x, y, yaw))
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        self.get_logger().info(
            f'Sending {len(poses)} WPs to stock Nav2 NavigateThroughPoses action')
        fut = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal rejected')
            return 1
        self.get_logger().info('goal accepted, awaiting result...')
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result()
        dur = int(time.time() - self.start)
        if result is None:
            self.get_logger().warn(f'RESULT: no-result duration {dur}s')
            return 1
        code = getattr(result.result, 'error_code', None)
        msg = getattr(result.result, 'error_msg', '')
        n_total = len(self.wps)
        n_remaining = self.last_remaining if self.last_remaining is not None else n_total
        n_reached = n_total - n_remaining
        self.get_logger().info(
            f'RESULT: reached {n_reached}/{n_total} '
            f'remaining {n_remaining} duration {dur}s '
            f'code={code} msg="{msg}"')
        return 0

    def _on_feedback(self, msg):
        fb = msg.feedback
        rem = int(getattr(fb, 'number_of_poses_remaining', -1))
        cp = fb.current_pose.pose.position if hasattr(fb, 'current_pose') else None
        dist = float(getattr(fb, 'distance_remaining', 0.0))
        dur = int(time.time() - self.start)
        if rem != self.last_remaining:
            self.last_remaining = rem
            cur = f'({cp.x:.1f},{cp.y:.1f})' if cp else ''
            self.get_logger().info(
                f'  WP progress  remaining={rem}  dist={dist:.1f}m  cur={cur}  t={dur}s')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--spacing', type=float, default=4.0)
    args = ap.parse_args()

    wps = subsample(args.trajectory, args.spacing)
    print(f'Subsampled to {len(wps)} WPs at {args.spacing}m spacing')

    rclpy.init()
    node = NavClient(wps)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)
