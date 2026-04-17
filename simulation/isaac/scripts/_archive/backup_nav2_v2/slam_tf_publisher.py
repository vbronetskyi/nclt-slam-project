#!/usr/bin/env python3
"""
bridges ORB-SLAM3 pose to Nav2 TF: publishes map->odom correction.

subscribes to /clock from Isaac Sim and uses those timestamps for TF,
ensuring all transforms are in the same time domain (sim time).
Nav2 must run with use_sim_time=true.

usage:
  source /opt/ros/jazzy/setup.bash
  python3 slam_tf_publisher.py [--use-gt]
"""
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class SlamTFPublisher(Node):
    def __init__(self, use_gt=False):
        # use_sim_time=true so get_clock().now() returns /clock time
        super().__init__('slam_tf_publisher',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.br = TransformBroadcaster(self)
        self.use_gt = use_gt
        self.pose_file = '/tmp/slam_pose.txt'

        self.world_x0 = -95.0
        self.world_y0 = -6.0
        self.world_yaw0 = 0.0
        self.slam_origin = None

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_received = False

        # track latest clock from Isaac Sim for TF timestamps
        self.latest_clock = None
        self.sub_clock = self.create_subscription(Clock, '/clock', self.clock_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.publish_tf)  # 20 Hz

        mode = "GT (no SLAM)" if use_gt else "SLAM"
        self.get_logger().info(f'TF publisher started, mode={mode}, using /clock timestamps')

    def clock_cb(self, msg):
        self.latest_clock = msg.clock

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_x = p.x
        self.odom_y = p.y
        self.odom_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def publish_tf(self):
        if not self.odom_received or self.latest_clock is None:
            return
        # skip if clock is still at 0 (Isaac Sim not yet running)
        if self.latest_clock.sec == 0 and self.latest_clock.nanosec == 0:
            return

        if self.use_gt:
            self._send_tf(0.0, 0.0, 0.0, 0.0)
            return

        try:
            with open(self.pose_file, 'r') as f:
                line = f.readline().strip()
            parts = line.split()
            if len(parts) < 8:
                return

            sx = float(parts[1])
            sy = float(parts[2])
            sz = float(parts[3])
            qx = float(parts[4])
            qy = float(parts[5])
            qz = float(parts[6])
            qw = float(parts[7])
            slam_yaw = yaw_from_quat(qx, qy, qz, qw)

            if self.slam_origin is None:
                self.slam_origin = (sx, sy, sz, slam_yaw)
                self.get_logger().info(
                    f'SLAM origin: ({sx:.3f},{sy:.3f},{sz:.3f}) yaw={slam_yaw:.3f}')

            s0x, s0y, s0z, s0yaw = self.slam_origin
            dx_s = sx - s0x
            dy_s = sy - s0y
            dz_s = sz - s0z
            dyaw = slam_yaw - s0yaw

            cos_g = math.cos(self.world_yaw0)
            sin_g = math.sin(self.world_yaw0)
            slam_wx = self.world_x0 + dz_s * cos_g + dx_s * sin_g
            slam_wy = self.world_y0 + dz_s * sin_g - dx_s * cos_g
            slam_wyaw = self.world_yaw0 - dyaw

            co = math.cos(-self.odom_yaw)
            so = math.sin(-self.odom_yaw)
            inv_ox = -(self.odom_x * co - self.odom_y * so)
            inv_oy = -(self.odom_x * so + self.odom_y * co)
            inv_oyaw = -self.odom_yaw

            cs = math.cos(slam_wyaw)
            ss = math.sin(slam_wyaw)
            mo_x = slam_wx + inv_ox * cs - inv_oy * ss
            mo_y = slam_wy + inv_ox * ss + inv_oy * cs
            mo_yaw = slam_wyaw + inv_oyaw

            self._send_tf(mo_x, mo_y, 0.0, mo_yaw)

        except (FileNotFoundError, ValueError, IndexError):
            pass

    def _send_tf(self, x, y, z, yaw):
        t = TransformStamped()
        # use /clock timestamp from Isaac Sim (sim time)
        t.header.stamp = self.latest_clock
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.br.sendTransform(t)


def main():
    use_gt = '--use-gt' in sys.argv
    rclpy_argv = [a for a in sys.argv if a != '--use-gt']
    rclpy.init(args=rclpy_argv)
    node = SlamTFPublisher(use_gt=use_gt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
