#!/usr/bin/env python3
"""Drive a route point-to-point, record trajectory + rosbag for SLAM replay

Usage: python3 drive_route.py [1|2|3]

Records:
  - /tmp/trajectory_r{N}.csv - timestamped x,y,yaw,pitch,roll,imu_ax,imu_ay,imu_az
  - rosbag in /workspace/simulation/bags/route_{N}/ - camera, IMU, lidar, odom, tf
"""
import rclpy, math, sys, time, subprocess, os, json
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

SPAWN = (-170, 0)


class RouteDriver(Node):
    def __init__(self, waypoints_world, spawn_override=None, route_id=1):
        super().__init__('route_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.ox = self.oy = self.yaw = None
        self.pitch = self.roll = 0.0
        self.imu_ax = self.imu_ay = self.imu_az = 0.0
        self.spawn = spawn_override or SPAWN
        self.gt_offset_x = spawn_override[0] if spawn_override else SPAWN[0]
        self.gt_offset_y = spawn_override[1] if spawn_override else SPAWN[1]
        self.odom_x = None
        self.odom_y = None
        self.wps = waypoints_world
        self.wp_idx = 0
        self.done = False
        self.timer = self.create_timer(0.1, self.control_loop)
        self.gt_sub = self.create_subscription(
            TFMessage, '/gz_poses', self.gz_poses_cb, 10)  # GT ~50Hz
        self.last_print = 0
        self.last_move_time = time.time()
        self.last_move_x = None
        self.last_move_y = None
        self.finish_status = 'INCOMPLETE'

        # trajectory log
        traj_path = f'/tmp/trajectory_r{route_id}.csv'
        self.traj_file = open(traj_path, 'w')
        self.traj_file.write('time,wx,wy,yaw,pitch,roll,imu_ax,imu_ay,imu_az\n')
        self.traj_count = 0
        print(f'Trajectory log: {traj_path}')

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = 2 * math.atan2(q.z, q.w)
        # odom + GT offset for world pos
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.ox = self.odom_x + self.gt_offset_x
        self.oy = self.odom_y + self.gt_offset_y
        try:  # write pos for other scripts to read
            with open('/tmp/robot_pos.txt', 'w') as f:
                f.write(f'{self.ox:.2f} {self.oy:.2f} {self.yaw:.4f} {self.pitch:.4f} {self.roll:.4f}')
        except:
            pass
        self.traj_count += 1
        if self.traj_count % 5 == 0:
            self.traj_file.write(
                f'{time.time():.3f},{self.ox:.3f},{self.oy:.3f},{self.yaw:.4f},'
                f'{self.pitch:.4f},{self.roll:.4f},'
                f'{self.imu_ax:.4f},{self.imu_ay:.4f},{self.imu_az:.4f}\n')

    def gz_poses_cb(self, msg):
        """GT from /gz_poses - match robot by expected position"""
        if self.odom_x is None:
            return
        # expected world pos via odom
        expect_x = self.odom_x + self.gt_offset_x
        expect_y = self.odom_y + self.gt_offset_y

        best_d = 30.0  # 30m search radius
        best_x = best_y = None
        for t in msg.transforms:
            gx = t.transform.translation.x
            gy = t.transform.translation.y
            gz = t.transform.translation.z
            if gz < 1.0 or gz > 10.0:  # robot Z ~3m (terrain height)
                continue
            d = math.hypot(gx - expect_x, gy - expect_y)
            if d < best_d:
                best_d = d
                best_x, best_y = gx, gy

        if best_x is not None:
            new_off_x = best_x - self.odom_x
            new_off_y = best_y - self.odom_y
            drift = math.hypot(new_off_x - self.gt_offset_x, new_off_y - self.gt_offset_y)
            if drift > 0.3:
                self.get_logger().info(f'GT fix: drift={drift:.1f}m, real=({best_x:.0f},{best_y:.0f})')
            self.gt_offset_x = new_off_x
            self.gt_offset_y = new_off_y

    def imu_cb(self, msg):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(max(-1, min(1, sinp)))
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.imu_az = msg.linear_acceleration.z

    def world_pos(self):
        if self.ox is None:
            return None, None
        return self.ox, self.oy  # ground truth = already world coords

    def control_loop(self):
        if self.done or self.ox is None:
            return

        # stuck detection - check actual movement, not just wheel spin
        wx_t, wy_t = self.wps[self.wp_idx]
        curr_dist_to_wp = math.hypot(wx_t - self.ox, wy_t - self.oy)

        if self.last_move_x is not None:
            moved = math.hypot(self.ox - self.last_move_x, self.oy - self.last_move_y)
            if moved > 1.0:  # 1m = meaningful movement
                self.last_move_time = time.time()
                self.last_move_x = self.ox
                self.last_move_y = self.oy
            elif time.time() - self.last_move_time > 10:
                wpos = self.world_pos()
                print(f'\n  STUCK at ({wpos[0]:.0f},{wpos[1]:.0f}) - no progress for 10s')
                print(f'  Distance to WP{self.wp_idx+1}: {curr_dist_to_wp:.0f}m')
                self.stop()
                self.done = True
                self.finish_status = 'STUCK'
                return
        else:
            self.last_move_x = self.ox
            self.last_move_y = self.oy

        wx, wy = self.wps[self.wp_idx]
        # ox/oy already in world coords (GT-corrected)
        dx, dy = wx - self.ox, wy - self.oy
        dist = math.hypot(dx, dy)

        now = time.time()
        if now - self.last_print > 5:
            wpos = self.world_pos()
            self.get_logger().info(
                f'WP{self.wp_idx+1}/{len(self.wps)} target({wx},{wy}) '
                f'pos({wpos[0]:.0f},{wpos[1]:.0f}) dist={dist:.0f}m')
            self.last_print = now

        # 3m acceptance radius for waypoint
        if dist < 3.0:
            wpos = self.world_pos()
            print(f'  WP{self.wp_idx+1}/{len(self.wps)} ({wx},{wy}): '
                  f'pos({wpos[0]:.0f},{wpos[1]:.0f}) dist={dist:.0f}m [OK]')
            self.wp_idx += 1
            if self.wp_idx >= len(self.wps):
                self.finish()
                return
            return

        desired = math.atan2(dy, dx)
        err = desired - self.yaw
        while err > math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi

        cmd = Twist()
        if abs(err) > 0.3:
            # big heading error, turn in place
            cmd.linear.x = 0.05
            cmd.angular.z = max(-1.0, min(1.0, err * 2.0))
        elif abs(err) > 0.1:
            cmd.linear.x = 0.3
            cmd.angular.z = max(-0.5, min(0.5, err * 1.2))
        else:
            cmd.linear.x = 0.5
            cmd.angular.z = max(-0.3, min(0.3, err * 0.8))
        self.pub.publish(cmd)

    def finish(self):
        self.stop()
        self.done = True
        wpos = self.world_pos()
        d = math.hypot(100 - wpos[0], 0 - wpos[1])
        if d < 15:
            self.finish_status = 'SUCCESS'
        else:
            self.finish_status = 'PARTIAL'
        print(f'\nFINAL: world({wpos[0]:.0f},{wpos[1]:.0f}), dist to village: {d:.0f}m')
        print(self.finish_status)
        self.traj_file.close()
        print(f'Trajectory saved to /tmp/trajectory_r{idx}.csv')

    def stop(self):
        cmd = Twist()
        self.pub.publish(cmd)


# load routes from json
_rd = json.load(open('/workspace/simulation/routes/routes.json'))


def _mk(key):
    r = _rd[key]
    return [tuple(w) for w in r['waypoints']] + [tuple(r['goal'])]


r1 = _mk('route_1')
r2 = _mk('route_2')
r3 = _mk('route_3')

routes = [
    (f"ROUTE 1: {_rd['route_1']['name']}", r1, tuple(_rd['route_1']['start'])),
    (f"ROUTE 2: {_rd['route_2']['name']}", r2, tuple(_rd['route_2']['start'])),
    (f"ROUTE 3: {_rd['route_3']['name']}", r3, tuple(_rd['route_3']['start'])),
]

idx = int(sys.argv[1]) if len(sys.argv) > 1 else 1
rname, wps, spawn = routes[idx - 1]

print(f'\n{"=" * 60}')
print(f'{rname}')
print(f'Start: {spawn}, {len(wps)} waypoints, Goal: ({wps[-1][0]},{wps[-1][1]})')
print(f'{"=" * 60}')

# start rosbag recording
bag_dir = f'/workspace/simulation/bags/route_{idx}'
os.makedirs(bag_dir, exist_ok=True)
bag_topics = '/odom /imu/data /scan /camera/color/image_raw /camera/depth/image_rect_raw /camera/camera_info /tf /clock'
print(f'Recording rosbag to {bag_dir}...')
bag_proc = subprocess.Popen(
    f'source /opt/ros/jazzy/setup.bash && ros2 bag record -o {bag_dir} {bag_topics}',
    shell=True, executable='/bin/bash',
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# teleport to spawn - odom doesn't reset between runs
GZ = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"
yaw = {1: 0.0, 2: 0.4, 3: -0.8}.get(idx, 0.0)
spawn_z = 4.0  # safe height, will drop to surface
print(f'Teleporting to ({spawn[0]}, {spawn[1]})...')
subprocess.run([GZ, "service", "-s", "/world/outdoor_terrain/set_pose",
                "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                "--req", f'name: "husky_a200" position {{ x: {spawn[0]} y: {spawn[1]} z: {spawn_z} }} '
                         f'orientation {{ x: 0 y: 0 z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)} }}',
                "--timeout", "5000"], capture_output=True, timeout=10)
time.sleep(5)  # settle on terrain

# clear web trail after teleporting
try:
    import urllib.request
    urllib.request.urlopen('http://localhost:8765/clear_trail', timeout=2)
    print('Web trail cleared')
except:
    pass

# read odom to compute offset - doesn't reset on teleport
rclpy.init()
print('Reading odom offset after teleport...')

import threading
_odom_ox = [None]
_odom_oy = [None]
_odom_evt = threading.Event()

class _OdomReader(Node):
    def __init__(self):
        super().__init__('_odom_reader')
        self.sub = self.create_subscription(Odometry, '/odom', self._cb, 10)
    def _cb(self, msg):
        _odom_ox[0] = msg.pose.pose.position.x
        _odom_oy[0] = msg.pose.pose.position.y
        _odom_evt.set()

_reader = _OdomReader()
for _ in range(100):
    rclpy.spin_once(_reader, timeout_sec=0.1)
    if _odom_evt.is_set():
        break
_reader.destroy_node()

if _odom_ox[0] is not None:
    # spawn_offset = actual spawn - odom reading after teleport
    actual_spawn = (spawn[0] - _odom_ox[0], spawn[1] - _odom_oy[0])
    print(f'Odom after teleport: ({_odom_ox[0]:.1f}, {_odom_oy[0]:.1f}) -> offset: ({actual_spawn[0]:.1f}, {actual_spawn[1]:.1f})')
else:
    actual_spawn = spawn
    print(f'No odom received, using default spawn offset')

node = RouteDriver(wps, spawn_override=actual_spawn, route_id=idx)
try:
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)
except KeyboardInterrupt:
    pass

node.stop()
node.traj_file.close()
node.destroy_node()
rclpy.shutdown()

# stop rosbag recording
bag_proc.terminate()
bag_proc.wait(timeout=5)
print(f'Rosbag saved to {bag_dir}')
