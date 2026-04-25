#!/usr/bin/env python3
"""First autonomous navigation goal test
Robot: Husky A200 sim, SLAM Toolbox + Nav2
Goal: navigate from (0,0) to (5,5) in outdoor terrain
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time, math

class NavGoalTest(Node):
    def __init__(self):
        super().__init__('nav_goal_test')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._current_pose = None
        self._start_time = None

    def _odom_cb(self, msg):
        self._current_pose = msg.pose.pose

    def send_goal(self, x, y):
        self.get_logger().info('Waiting for Nav2...')
        self._client.wait_for_server()
        self.get_logger().info('Nav2 ready!')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        self._start_time = time.time()
        self.get_logger().info(f'Sending goal: ({x}, {y})')
        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            return 'REJECTED'

        self.get_logger().info('Goal ACCEPTED')
        result_future = goal_handle.get_result_async()

        timeout = 120
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=1.0)
            elapsed = time.time() - self._start_time
            if elapsed > timeout:
                self.get_logger().warn(f'TIMEOUT after {timeout}s - cancelling')
                goal_handle.cancel_goal_async()
                return 'TIMEOUT'

        elapsed = time.time() - self._start_time
        status = result_future.result().status
        self.get_logger().info(f'Result status: {status}, time: {elapsed:.1f}s')

        if self._current_pose:
            px = self._current_pose.position.x
            py = self._current_pose.position.y
            dist = math.sqrt((px-x)**2 + (py-y)**2)
            self.get_logger().info(f'Final position: ({px:.2f}, {py:.2f}), dist to goal: {dist:.2f}m')

        return 'SUCCESS' if status == 4 else f'STATUS_{status}'

    def _feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().info(f'  Distance remaining: {dist:.2f}m')

def main():
    rclpy.init()
    node = NavGoalTest()
    result = node.send_goal(5.0, 5.0)
    print(f'\n=== NAVIGATION RESULT: {result} ===\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
