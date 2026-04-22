#!/usr/bin/env python3
"""
two-phase waypoint sender for Nav2.

phase 1 (outbound): send waypoints start -> destination (obstacles present)
phase 2 (return):   signal obstacle removal, send waypoints destination -> start

communicates with run_husky_nav2.py via /tmp/nav2_phase.json.

usage:
  source /opt/ros/jazzy/setup.bash
  python3 send_nav2_goal.py --route road
"""
import json
import math
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


PHASE_FILE = "/tmp/nav2_phase.json"


def write_phase(phase):
    with open(PHASE_FILE, 'w') as f:
        json.dump({"phase": phase, 'timestamp': time.time()}, f)


def make_pose(x, y, yaw):
    # TODO: tune per route
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


class Nav2TwoPhase(Node):
    def __init__(self, route):
        super().__init__('nav2_goal_sender')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.route = route
        self.outbound_poses = []
        self.return_poses = []
        self.current_poses = []
        self.current_idx = 0
        self.phase = "outbound"
        self._retries = 0

    def start(self):
        with open('/tmp/slam_routes.json') as f:
            routes = json.load(f)

        wps = routes[self.route]

        # find turn point (closest to destination)
        dists = [math.hypot(p[0] - 72, p[1] + 5) for p in wps]
        turn_idx = dists.index(min(dists))

        # outbound: skip start, go to turn point (every waypoint for tight tracking)
        outbound_wps = wps[1:turn_idx + 1]
        for i, (x, y) in enumerate(outbound_wps):
            if i < len(outbound_wps) - 1:
                dx = outbound_wps[i + 1][0] - x
                dy = outbound_wps[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = math.pi  # face back at destination
            self.outbound_poses.append(make_pose(x, y, yaw))

        # return: reverse of outbound, back to start
        return_wps = list(reversed(wps[0:turn_idx + 1]))
        for i, (x, y) in enumerate(return_wps):
            if i < len(return_wps) - 1:
                dx = return_wps[i + 1][0] - x
                dy = return_wps[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
            self.return_poses.append(make_pose(x, y, yaw))

        self.get_logger().info(
            f'route "{self.route}": {len(self.outbound_poses)} outbound, '
            f'{len(self.return_poses)} return waypoints')

        # start outbound
        self.current_poses = self.outbound_poses
        self.current_idx = 0
        self.phase = "outbound"

        self.get_logger().info('waiting for Nav2 action server...')
        self.client.wait_for_server()
        self.get_logger().info('=== PHASE 1: OUTBOUND (obstacles present) ===')
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_idx >= len(self.current_poses):
            if self.phase == "outbound":
                self.get_logger().info('=== OUTBOUND COMPLETE ===')
                self.get_logger().info('signaling obstacle removal...')
                write_phase("removing")
                # wait for Isaac Sim to remove obstacles and write "return"
                time.sleep(5.0)
                self.get_logger().info('=== PHASE 2: RETURN (obstacles removed) ===')
                self.phase = "return"
                self.current_poses = self.return_poses
                self.current_idx = 0
                self._retries = 0
                self.send_next_goal()
            else:
                self.get_logger().info('=== ALL WAYPOINTS REACHED - NAVIGATION COMPLETE ===')
                write_phase("done")
            return

        pose = self.current_poses[self.current_idx]
        self.get_logger().info(
            f'[{self.phase}][{self.current_idx + 1}/{len(self.current_poses)}] goal: '
            f'({pose.pose.position.x:.1f},{pose.pose.position.y:.1f})')

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'goal {self.current_idx + 1} rejected, skipping...')
            self.current_idx += 1
            self._retries = 0
            self.send_next_goal()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        pos = fb.current_pose.pose.position
        self.get_logger().info(
            f'[{self.phase}][{self.current_idx + 1}/{len(self.current_poses)}] '
            f'pos=({pos.x:.1f},{pos.y:.1f}) remaining={fb.distance_remaining:.1f}m',
            throttle_duration_sec=5.0)

    def result_cb(self, future):
        result = future.result().result
        if result.error_code == 0:
            # print("DEBUG: isaac sim step")
            self.get_logger().info(
                f'[{self.phase}] waypoint {self.current_idx + 1} reached!')
            self.current_idx += 1
            self._retries = 0
            self.send_next_goal()
        else:
            self._retries += 1
            if self._retries >= 5:
                # print(f"DEBUG state={state} pose={pose}")
                self.get_logger().warn(
                    f'[{self.phase}] waypoint {self.current_idx + 1} failed {self._retries}x, skipping')
                self.current_idx += 1
                self._retries = 0
                self.send_next_goal()
            else:
                self.get_logger().info(
                    f'[{self.phase}] waypoint {self.current_idx + 1} failed '
                    f'(err={result.error_code}), retry {self._retries}/5...')
                time.sleep(2.0)
                self.send_next_goal()


def main():
    # parse route from argv manually to avoid argparse eating --ros-args
    route = 'road'
    for i, arg in enumerate(sys.argv):
        if arg == '--route' and i + 1 < len(sys.argv):
            route = sys.argv[i + 1]

    rclpy.init(args=sys.argv)
    node = Nav2TwoPhase(route)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
