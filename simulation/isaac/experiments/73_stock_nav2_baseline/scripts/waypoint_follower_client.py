#!/usr/bin/env python3
"""Stock-Nav2 baseline client v2 - via FollowWaypoints action.

The canonical Nav2 way to "visit a list of waypoints with skip-on-failure"
is the WaypointFollower action (stop_on_failure: false). Each WP is
delegated to NavigateToPose internally; if one fails, the follower logs it
and moves to the next.

Compared to v1 (NavigateThroughPoses single-trajectory) this survives
individual WP failures instead of aborting the whole route.
"""
import argparse, csv, math, time, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


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


class WaypointClient(Node):
    def __init__(self, wps):
        super().__init__('waypoint_follower_client')
        self.wps = wps
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.last_idx = -1
        self.start = time.time()

    def run(self):
        self.get_logger().info(f'Waiting for /follow_waypoints server...')
        if not self.client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('FollowWaypoints server did not come up')
            return 1
        poses = []
        for i, (x, y) in enumerate(self.wps):
            yaw = 0.0
            if i + 1 < len(self.wps):
                nx, ny = self.wps[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            poses.append(pose_from_xy(x, y, yaw))
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self.get_logger().info(
            f'Sending {len(poses)} WPs to stock Nav2 FollowWaypoints action')
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
        missed = list(getattr(result.result, 'missed_waypoints', []))
        n_total = len(self.wps)
        n_skipped = len(missed)
        n_reached = n_total - n_skipped
        self.get_logger().info(
            f'RESULT: reached {n_reached}/{n_total} '
            f'skipped {n_skipped} duration {dur}s '
            f'missed_idx={missed}')
        return 0

    def _on_feedback(self, msg):
        fb = msg.feedback
        idx = int(getattr(fb, 'current_waypoint', -1))
        dur = int(time.time() - self.start)
        if idx != self.last_idx:
            self.last_idx = idx
            x, y = self.wps[idx] if 0 <= idx < len(self.wps) else (0.0, 0.0)
            self.get_logger().info(
                f'  WP progress  idx={idx}/{len(self.wps)-1}  '
                f'target=({x:.1f},{y:.1f})  t={dur}s')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--spacing', type=float, default=4.0)
    args = ap.parse_args()

    wps = subsample(args.trajectory, args.spacing)
    print(f'Subsampled to {len(wps)} WPs at {args.spacing}m spacing')

    rclpy.init()
    node = WaypointClient(wps)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)
