#!/usr/bin/env python3
"""Build SLAM map by driving in expanding squares, then navigate to a safe goal nearby"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import time
import math
import numpy as np


class MapAndNavigate(Node):
    def __init__(self):
        super().__init__('map_and_navigate')
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._map_msg = None
        self._free_cells = 0
        self._pose = None

    def _map_cb(self, msg):
        self._map_msg = msg
        self._free_cells = sum(1 for c in msg.data if c == 0)

    def _odom_cb(self, msg):
        self._pose = msg.pose.pose

    def drive(self, linear, angular, duration):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        end = time.time() + duration
        while time.time() < end:
            self._cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def stop(self):
        self._cmd_pub.publish(Twist())
        time.sleep(0.5)

    def build_map(self, target_free_cells=15000):
        self.get_logger().info(f'=== Phase 1: Building map (target {target_free_cells} free cells) ===')

        loop = 0
        while self._free_cells < target_free_cells and loop < 8:
            loop += 1
            scale = 1.0 + loop * 0.3  # expand each loop
            self.get_logger().info(
                f'Loop {loop}, free cells: {self._free_cells}, scale: {scale:.1f}')

            # forward-turn pattern (expanding square)
            for _ in range(4):
                self.drive(0.5, 0.0, 2.5 * scale)  # forward
                self.drive(0.0, 0.7, 2.2)            # turn ~90deg
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info(f'Map built: {self._free_cells} free cells')
        if self._pose:
            self.get_logger().info(
                f'Robot at: ({self._pose.position.x:.2f}, {self._pose.position.y:.2f})')

    def find_safe_goal(self, min_dist=2.0, max_dist=3.5):
        """Find a free cell in the map within given distance from robot"""
        if self._map_msg is None or self._pose is None:
            self.get_logger().error('No map or pose available')
            return None

        m = self._map_msg
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        res = m.info.resolution
        w = m.info.width
        h = m.info.height
        data = np.array(m.data, dtype=np.int8).reshape((h, w))

        rx = self._pose.position.x
        ry = self._pose.position.y

        # robot pos -> grid coords
        rcol = int((rx - ox) / res)
        rrow = int((ry - oy) / res)

        best = None
        best_dist = 0.0

        # search in a ring around the robot
        min_cells = int(min_dist / res)
        max_cells = int(max_dist / res)

        for angle_deg in range(0, 360, 15):
            angle = math.radians(angle_deg)
            for r in range(min_cells, max_cells + 1, 2):
                col = rcol + int(r * math.cos(angle))
                row = rrow + int(r * math.sin(angle))
                if 0 <= col < w and 0 <= row < h:
                    # cell + 3x3 neighborhood all gotta be free
                    if data[row, col] == 0:
                        patch = data[max(0, row-2):row+3, max(0, col-2):col+3]
                        if patch.size > 0 and np.all(patch == 0):
                            gx = ox + col * res
                            gy = oy + row * res
                            d = math.sqrt((gx - rx)**2 + (gy - ry)**2)
                            if d > best_dist:
                                best = (gx, gy)
                                best_dist = d

        if best:
            self.get_logger().info(
                f'Found safe goal: ({best[0]:.2f}, {best[1]:.2f}), {best_dist:.2f}m from robot')
        else:
            self.get_logger().warn('No safe goal found in mapped area!')
        return best

    def navigate_to(self, x, y):
        self.get_logger().info(f'=== Phase 3: Navigating to ({x:.2f}, {y:.2f}) ===')
        self._client.wait_for_server(timeout_sec=10.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        start = time.time()
        last_log = start
        future = self._client.send_goal_async(goal, feedback_callback=self._fb_cb)
        rclpy.spin_until_future_complete(self, future)

        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal REJECTED')
            return 'REJECTED'

        self.get_logger().info('Goal ACCEPTED - navigating...')
        res_future = gh.get_result_async()
        while not res_future.done():
            rclpy.spin_once(self, timeout_sec=1.0)
            if time.time() - start > 120:
                self.get_logger().warn('TIMEOUT 120s - cancelling')
                gh.cancel_goal_async()
                return 'TIMEOUT'

        elapsed = time.time() - start
        status = res_future.result().status
        if self._pose:
            px, py = self._pose.position.x, self._pose.position.y
            dist = math.sqrt((px - x)**2 + (py - y)**2)
            self.get_logger().info(
                f'Final pos: ({px:.2f}, {py:.2f}), dist to goal: {dist:.2f}m, '
                f'time: {elapsed:.1f}s, status: {status}')
        return 'SUCCESS' if status == 4 else f'STATUS_{status}'

    def _fb_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        # throttle feedback to 1/s
        if not hasattr(self, '_last_fb') or time.time() - self._last_fb > 1.0:
            self.get_logger().info(f'  dist remaining: {dist:.2f}m')
            self._last_fb = time.time()


def main():
    rclpy.init()
    node = MapAndNavigate()

    # phase 1: build the map
    node.build_map(target_free_cells=15000)
    time.sleep(2.0)

    # phase 2: find a safe goal
    goal = node.find_safe_goal(min_dist=2.0, max_dist=3.5)
    if goal is None:
        # fallback - try shorter range
        goal = node.find_safe_goal(min_dist=1.0, max_dist=2.0)

    if goal is None:
        print('\n=== RESULT: NO SAFE GOAL FOUND ===\n')
        node.destroy_node()
        rclpy.shutdown()
        return

    # phase 3: go there
    result = node.navigate_to(goal[0], goal[1])
    print(f'\n=== NAVIGATION RESULT: {result} ===\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
