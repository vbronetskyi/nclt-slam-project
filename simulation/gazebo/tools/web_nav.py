#!/usr/bin/env python3
"""Web UI for UGV navigation: 2D map + camera + click-to-drive

GT from Gazebo dynamic_pose/info, goals via direct cmd_vel
"""

import threading, time, io, math, os, numpy as np, xml.etree.ElementTree as ET
# heads up: the web UI is pretty minimal, just goal click + live map. for real driving use rviz
import subprocess, re
from PIL import Image as PILImage, ImageDraw

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_msgs.msg import TFMessage

from flask import Flask, send_file, request, Response

app = Flask(__name__)

# global state
map_data = None
camera_jpeg = None
overhead_jpeg = None
robot_world = (-170.0, 0.0, 0.0)  # ground truth from Gazebo (x, y, yaw)
robot_pos = (0.0, 0.0, 0.0)       # map frame position (for nav goals)
nav_status = "idle"
goal_pos = None      # (x, y) in WORLD frame for map display
robot_trail = []     # [(x, y)] in WORLD frame (ground truth)
MAX_TRAIL = 2000
lock = threading.Lock()

WORLD_SIZE_X = 220.0
WORLD_SIZE_Y = 150.0
WORLD_SIZE = WORLD_SIZE_X  # for w2px X axis
MAP_PX = 1000  # width
MAP_PX_Y = int(MAP_PX * WORLD_SIZE_Y / WORLD_SIZE_X)  # height = 682

# load model positions from SDF
MODEL_POSITIONS = []
MODEL_COLORS = {
    'Oak': (34, 139, 34), 'Pine': (0, 100, 0), 'Rock': (139, 119, 101),
    'Barrel': (255, 165, 0), 'Cone': (255, 69, 0),
    'house': (180, 160, 130), 'collapsed': (140, 90, 80), 'ruin': (140, 90, 80),
    'lamp': (100, 100, 100), 'dumpster': (60, 80, 60),
    'debris': (150, 130, 110), 'jersey': (180, 175, 165),
    'fallen': (70, 45, 25), 'stump': (85, 55, 30), 'Bush': (50, 100, 30),
}

def load_model_positions():
    sdf = '/workspace/simulation/src/ugv_gazebo/worlds/outdoor_terrain.sdf'
    try:
        tree = ET.parse(sdf)
        for inc in tree.iter('include'):
            name = inc.find('name')
            pose = inc.find('pose')
            if name is not None and pose is not None:
                p = [float(x) for x in pose.text.split()]
                n = name.text
                color = (128, 128, 128)
                for key, c in MODEL_COLORS.items():
                    if key.lower() in n.lower():
                        color = c
                        break
                yaw = p[5] if len(p) > 5 else 0.0
                MODEL_POSITIONS.append((n, p[0], p[1], color, yaw))
    except:
        pass

load_model_positions()


# Gazebo ground truth pose
def odom_tracking_thread():
    """Odom -> world coords via spawn offset - drifts but updates fast"""
    global robot_world, robot_trail
    import rclpy as rclpy2
    from nav_msgs.msg import Odometry

    SPAWN_X, SPAWN_Y = -105.0, 0.0  # must match launch file

    class OdomTracker(Node):
        def __init__(self):
            super().__init__('odom_tracker')
            self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)

        def cb(self, msg):
            global robot_world, robot_trail
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            # odom -> world (approximate, odom starts at spawn)
            wx = ox + SPAWN_X
            wy = oy + SPAWN_Y
            with lock:
                robot_world = (wx, wy, yaw)
                if not robot_trail or ((wx - robot_trail[-1][0])**2 + (wy - robot_trail[-1][1])**2) > 0.25:
                    robot_trail.append((wx, wy))
                    if len(robot_trail) > MAX_TRAIL:
                        robot_trail = robot_trail[-MAX_TRAIL:]
            try:
                tmp = '/tmp/robot_pos.txt.tmp'
                with open(tmp, 'w') as f:
                    f.write(f'{wx:.2f} {wy:.2f} {yaw:.4f} 0 0')
                os.replace(tmp, '/tmp/robot_pos.txt')
            except:
                pass

    # spin in this thread
    node = OdomTracker()
    while rclpy2.ok():
        rclpy2.spin_once(node, timeout_sec=0.1)
    node.destroy_node()


class NavWebNode(Node):
    def __init__(self):
        super().__init__('nav_web')
        # SLAM map disabled - we don't have LiDAR
        #self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.cam_cb, 1)
        self.overhead_sub = self.create_subscription(Image, '/chase_camera/image', self.overhead_cb, 1)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, odom_qos)
        # gT from dynamic_pose - only moving models = our robot
        gt_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.gt_sub = self.create_subscription(TFMessage, '/ground_truth/dynamic_poses', self.gt_dynamic_cb, gt_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._drive_cancel = False
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.update_tf)

    def map_cb(self, msg):
        global map_data
        with lock:
            map_data = msg

    def cam_cb(self, msg):
        global camera_jpeg
        # no skip needed - camera only +-1-2 Hz anyway
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = PILImage.fromarray(arr[:, :, :3])
        img = img.resize((320, 240), PILImage.BILINEAR)
        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=40)
        with lock:
            camera_jpeg = buf.getvalue()

    def overhead_cb(self, msg):
        global overhead_jpeg
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            img = PILImage.fromarray(arr[:, :, :3])
            img = img.resize((320, 240), PILImage.BILINEAR)
            buf = io.BytesIO()
            img.save(buf, format='JPEG', quality=40)
            with lock:
                overhead_jpeg = buf.getvalue()
        except:
            pass

    def spawn_offset_recalib(self, real_x, real_y):
        """Recalibrate odom offset after teleport"""
        self._recalib_target = (real_x, real_y)
        self._recalib_pending = True

    def odom_cb(self, msg):
        """Odom + spawn offset -> world pos, writes /tmp/robot_pos.txt"""
        global robot_world, robot_trail
        # recalibrate if we just teleported
        if hasattr(self, '_recalib_pending') and self._recalib_pending:
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            self._spawn_x = self._recalib_target[0] - ox
            self._spawn_y = self._recalib_target[1] - oy
            self._recalib_pending = False
        if not hasattr(self, '_spawn_x'):
            self._spawn_x = -170.0
            self._spawn_y = 0.0
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        SPAWN_X = self._spawn_x
        SPAWN_Y = self._spawn_y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        wx = ox + SPAWN_X
        wy = oy + SPAWN_Y
        # median of last 5 readings - filters outliers
        if not hasattr(self, '_pos_buf'):
            self._pos_buf = []
        self._pos_buf.append((wx, wy))
        if len(self._pos_buf) > 5:
            self._pos_buf = self._pos_buf[-5:]
        xs = sorted(p[0] for p in self._pos_buf)
        ys = sorted(p[1] for p in self._pos_buf)
        wx = xs[len(xs)//2]  # median
        wy = ys[len(ys)//2]
        # actual pos from gt_dynamic_cb - odom is just for recalib + yaw fallback

    def gt_dynamic_cb(self, msg):
        """GT from /dynamic_pose/info - only moving models, first = robot chassis"""
        global robot_world, robot_trail
        if not msg.transforms:
            return
        # first transform is our robot base
        t = msg.transforms[0]
        gx = t.transform.translation.x
        gy = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with lock:
            if robot_trail:
                jump = math.hypot(gx - robot_trail[-1][0], gy - robot_trail[-1][1])
                if jump > 20:
                    robot_trail.clear()
            robot_world = (gx, gy, yaw)
            if not robot_trail or math.hypot(gx - robot_trail[-1][0], gy - robot_trail[-1][1]) > 0.5:
                robot_trail.append((gx, gy))
                if len(robot_trail) > MAX_TRAIL:
                    robot_trail = robot_trail[-MAX_TRAIL:]
            # atomic write - rename trick
            try:
                tmp = '/tmp/robot_pos.txt.tmp'
                with open(tmp, 'w') as f:
                    f.write(f'{gx:.2f} {gy:.2f} {yaw:.4f} 0 0')
                os.replace(tmp, '/tmp/robot_pos.txt')
            except:
                pass
            # append GT to trajectory csv
            try:
                with open('/tmp/gt_trajectory.csv', 'a') as f:
                    f.write(f'{time.time():.3f},{gx:.4f},{gy:.4f},{yaw:.4f}\n')
            except:
                pass

    def gz_poses_cb(self, msg):
        """DISABLED - replaced by gt_dynamic_cb"""
        pass

    def _old_gz_poses_cb(self, msg):
        """Old GT method - find entity closest to last known pos"""
        global robot_world, robot_trail

        # only stuff at terrain height (+-1.5-8m Z)
        curr = []
        for t in msg.transforms:
            gx = t.transform.translation.x
            gy = t.transform.translation.y
            gz_z = t.transform.translation.z
            if 1.5 < gz_z < 8.0:
                curr.append((gx, gy))

        # find robot - closest to last known pos
        with lock:
            prev_x, prev_y, ryaw = robot_world

        best_d = 50.0
        best_x = best_y = None
        for gx, gy in curr:
            d = math.hypot(gx - prev_x, gy - prev_y)
            if d < best_d:
                best_d = d
                best_x, best_y = gx, gy

        if best_x is not None:
            with lock:
                _, _, ryaw = robot_world
                if robot_trail:
                    jump = math.hypot(best_x - robot_trail[-1][0], best_y - robot_trail[-1][1])
                    if jump > 20:
                        robot_trail.clear()
                robot_world = (best_x, best_y, ryaw)
                if not robot_trail or math.hypot(best_x - robot_trail[-1][0], best_y - robot_trail[-1][1]) > 0.5:
                    robot_trail.append((best_x, best_y))
                    if len(robot_trail) > MAX_TRAIL:
                        robot_trail = robot_trail[-MAX_TRAIL:]
                try:
                    with open('/tmp/robot_pos.txt', 'w') as f:
                        f.write(f'{best_x:.2f} {best_y:.2f} {ryaw:.4f} 0 0')
                except:
                    pass

    def update_tf(self):
        """Cache map-frame robot pos for nav goals"""
        global robot_pos
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with lock:
                robot_pos = (x, y, yaw)
        except:
            pass

    def world_to_map(self, wx, wy):
        """World coords -> map frame via rotation + translation around robot"""
        with lock:
            mx, my, myaw = robot_pos
            rwx, rwy, ryaw = robot_world
        # rotation between map and world
        theta = myaw - ryaw
        ct, st = math.cos(theta), math.sin(theta)
        # robot-relative -> rotate -> map frame
        dx = wx - rwx
        dy = wy - rwy
        map_x = ct * dx - st * dy + mx
        map_y = st * dx + ct * dy + my
        return map_x, map_y

    def send_goal(self, world_x, world_y):
        """Direct cmd_vel drive to world coords (no Nav2)"""
        global nav_status, goal_pos
        # cancel any in-flight drive
        self._drive_cancel = True
        time.sleep(0.2)
        goal_pos = (world_x, world_y)
        nav_status = f"driving to ({world_x:.1f}, {world_y:.1f})"
        self._drive_cancel = False

        def direct_drive():
            global nav_status, goal_pos
            pub = self.create_publisher(Twist, '/cmd_vel', 10)
            while nav_status != "stopped" and not self._drive_cancel:
                with lock:
                    rwx, rwy, ryaw = robot_world
                dx, dy = world_x - rwx, world_y - rwy
                dist = math.hypot(dx, dy)
                if dist < 2.0:
                    pub.publish(Twist())
                    nav_status = "arrived!"
                    goal_pos = None
                    return
                nav_status = f"driving... ({dist:.0f}m left)"
                desired = math.atan2(dy, dx)
                err = desired - ryaw
                while err > math.pi: err -= 2*math.pi
                while err < -math.pi: err += 2*math.pi
                cmd = Twist()
                if abs(err) > 0.3:
                    cmd.linear.x = 0.1
                    cmd.angular.z = max(-1.0, min(1.0, err * 2.0))
                elif abs(err) > 0.1:
                    cmd.linear.x = 0.5
                    cmd.angular.z = max(-0.5, min(0.5, err * 1.2))
                else:
                    cmd.linear.x = 0.9
                    cmd.angular.z = max(-0.3, min(0.3, err * 0.8))
                pub.publish(cmd)
                time.sleep(0.1)
            pub.publish(Twist())
            goal_pos = None

        threading.Thread(target=direct_drive, daemon=True).start()

    def _send_single_goal(self, map_x, map_y, world_x, world_y):
        """Send Nav2 goal in map frame"""
        self._goal_done = False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(map_x)
        goal.pose.pose.position.y = float(map_y)

        with lock:
            rwx, rwy, _ = robot_world
        yaw = math.atan2(world_y - rwy, world_x - rwx)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        future = self.nav_client.send_goal_async(goal, feedback_callback=self.nav_feedback)
        future.add_done_callback(self._wp_goal_response_cb)

    def _wp_goal_response_cb(self, future):
        result = future.result()
        if result and result.accepted:
            result.get_result_async().add_done_callback(self._wp_result_cb)
        else:
            self._goal_done = True

    def _wp_result_cb(self, future):
        self._goal_done = True

    def nav_feedback(self, feedback_msg):
        pass

    def stop(self):
        global nav_status, goal_pos
        t = Twist()
        for _ in range(10):
            self.cmd_pub.publish(t)
            time.sleep(0.02)
        nav_status = "stopped"
        goal_pos = None


node = None


# heightmap terrain background
TERRAIN_IMG = None

def _load_terrain():
    global TERRAIN_IMG
    tex_path = '/workspace/simulation/src/ugv_gazebo/worlds/terrain_texture.png'
    try:
        img = PILImage.open(tex_path)
        # no flip - texture Y matches world Y after w2px
        # img = img.transpose(PILImage.FLIP_TOP_BOTTOM)
        img = img.resize((MAP_PX, MAP_PX_Y), PILImage.BILINEAR)
        TERRAIN_IMG = img
    except Exception as e:
        print(f"[warn] Could not load heightmap: {e}")

_load_terrain()


def render_map_image():
    """Render map with terrain, models, SLAM overlay, trail, robot, goal - all world frame"""
    with lock:
        m = map_data
        rx, ry, ryaw = robot_world
        mx_r, my_r, _ = robot_pos  # map frame robot (for SLAM overlay alignment)
        gp = goal_pos

    sx = MAP_PX
    sy = MAP_PX_Y
    sz = sx  # legacy
    half_x = WORLD_SIZE_X / 2.0
    half_y = WORLD_SIZE_Y / 2.0

    def world_to_px(wx, wy):
        px = int((wx + half_x) / WORLD_SIZE_X * sx)
        py = int((half_y - wy) / WORLD_SIZE_Y * sy)
        return px, py

    #map->world transform (inverse)
    with lock:
        _, _, myaw = robot_pos
        _, _, ryaw_w = robot_world
    theta = myaw - ryaw_w
    ct, st = math.cos(-theta), math.sin(-theta)

    def map_to_px(map_x, map_y):
        """SLAM map-frame -> pixels via world frame"""
        dx = map_x - mx_r
        dy = map_y - my_r
        wx = ct * dx - st * dy + rx
        wy = st * dx + ct * dy + ry
        return world_to_px(wx, wy)

    # terrain background   
    if TERRAIN_IMG is not None:
        img = TERRAIN_IMG.copy()
    else:
        img = PILImage.new('RGB', (sx, sy), (50, 60, 50))
    draw = ImageDraw.Draw(img)

    # SLAM occupancy grid
    if m is not None:
        mw, mh = m.info.width, m.info.height
        mres = m.info.resolution
        mox = m.info.origin.position.x
        moy = m.info.origin.position.y
        data = np.array(m.data, dtype=np.int8).reshape(mh, mw)

        overlay = PILImage.new('RGBA', (sx, sy), (0, 0, 0, 0))
        ov_draw = ImageDraw.Draw(overlay)

        # draw occupied cells
        occupied = np.argwhere(data > 50)
        for row, col in occupied:
            wx_m = mox + (col + 0.5) * mres
            wy_m = moy + (row + 0.5) * mres
            px, py = map_to_px(wx_m, wy_m)
            cell_px = max(1, int(mres / WORLD_SIZE * sz))
            hc = max(1, cell_px // 2)
            ov_draw.rectangle([px-hc, py-hc, px+hc, py+hc], fill=(30, 30, 30, 180))

        # explored boundary - free cells touching unknown
        free = (data == 0)
        unknown = (data == -1)
        from scipy.ndimage import binary_dilation
        boundary = free & binary_dilation(unknown, iterations=1)
        for row, col in np.argwhere(boundary):
            wx_m = mox + (col + 0.5) * mres
            wy_m = moy + (row + 0.5) * mres
            px, py = map_to_px(wx_m, wy_m)
            cell_px = max(1, int(mres / WORLD_SIZE * sz))
            hc = max(1, cell_px // 2)
            ov_draw.rectangle([px-hc, py-hc, px+hc, py+hc], fill=(100, 180, 255, 50))

        img = PILImage.alpha_composite(img.convert('RGBA'), overlay).convert('RGB')
        draw = ImageDraw.Draw(img)

    # SDF model markers
    for item in MODEL_POSITIONS:
        name, mx, my, color = item[0], item[1], item[2], item[3]
        model_yaw = item[4] if len(item) > 4 else 0.0
        px, py = world_to_px(mx, my)
        if 0 <= px < sz and 0 <= py < sz:
            r = 8
            nl = name.lower()
            if 'house' in nl or 'collapsed' in nl or 'ruin' in nl:
                s = 18
                draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(60,40,30), width=2)
            elif 'fallen' in nl:
                dx = int(10 * math.cos(model_yaw))
                dy = int(-10 * math.sin(model_yaw))
                draw.line([px-dx, py-dy, px+dx, py+dy], fill=(90, 60, 30), width=3)
            elif 'stump' in nl:
                draw.ellipse([px-4, py-4, px+4, py+4], fill=color, outline=(60,40,20))
            elif 'bush' in nl:
                draw.ellipse([px-5, py-5, px+5, py+5], fill=color, outline=(30,70,20))
            elif ('tree' in nl or 'pine' in nl or 'oak' in nl) and 'fallen' not in nl:
                draw.ellipse([px-r, py-r, px+r, py+r], fill=color, outline=(255,255,255), width=2)
                draw.ellipse([px-2, py-2, px+2, py+2], fill=(255,255,255))
            elif 'rock' in nl:
                draw.polygon([(px,py-r),(px+r,py),(px,py+r),(px-r,py)], fill=color, outline=(255,255,255))
            elif 'jersey' in nl:
                draw.rectangle([px-6, py-2, px+6, py+2], fill=color, outline=(140,140,130))
            elif 'dumpster' in nl:
                draw.rectangle([px-5, py-4, px+5, py+4], fill=color, outline=(40,60,40))
            elif 'debris' in nl:
                draw.polygon([(px,py-4),(px+5,py+2),(px-5,py+2)], fill=color, outline=(120,110,90))
            else:
                s = r // 2 + 1
                draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(255,255,255), width=2)

    # trail in GT world coords
    with lock:
        trail = list(robot_trail)
    if len(trail) > 1:
        pts = [world_to_px(tx, ty) for tx, ty in trail]
        for i in range(1, len(pts)):
            draw.line([pts[i-1], pts[i]], fill=(255, 200, 0), width=2)

    # goal X marker
    if gp:
        gx, gy = world_to_px(gp[0], gp[1])
        r = 10
        draw.line([gx-r, gy-r, gx+r, gy+r], fill=(255, 0, 0), width=3)
        draw.line([gx-r, gy+r, gx+r, gy-r], fill=(255, 0, 0), width=3)
        draw.ellipse([gx-r, gy-r, gx+r, gy+r], outline=(255, 0, 0), width=2)

    #robot dot (GT pos)
    rpx, rpy = world_to_px(rx, ry)
    r = 8
    draw.ellipse([rpx-r, rpy-r, rpx+r, rpy+r], fill=(0, 150, 255), outline=(255,255,255), width=2)
    dx = int(r * 2.5 * math.cos(ryaw))
    dy = int(-r * 2.5 * math.sin(ryaw))
    draw.line([rpx, rpy, rpx+dx, rpy+dy], fill=(255, 80, 80), width=3)

    # legend
    y0 = 10
    items = [("Robot", (0,150,255)), ("Goal", (255,0,0)),
             ("Tree", (34,139,34)), ("Rock", (139,119,101)),
             ("Building", (180,160,130)), ("Ruin", (140,90,80)),
             ("Barrel", (255,165,0)), ("Cone", (255,69,0)),
             ("Obstacle (SLAM)", (30,30,30)), ("Explored edge", (100,180,255))]
    for label, c in items:
        draw.rectangle([10, y0, 22, y0+12], fill=c)
        draw.text((26, y0), label, fill=(255,255,255))
        y0 += 16

    # scale bar and compass
    bar_m = 10
    bar_px = int(bar_m / WORLD_SIZE * sz)
    draw.rectangle([10, sz-25, 10+bar_px, sz-20], fill=(255,255,255))
    draw.text((10, sz-18), f"{bar_m}m", fill=(255,255,255))
    draw.text((sz//2-10, sz-16), "S", fill=(200,200,200))
    draw.text((sz//2-10, 2), "N", fill=(200,200,200))
    draw.text((2, sz//2-6), "W", fill=(200,200,200))
    draw.text((sz-12, sz//2-6), "E", fill=(200,200,200))

    return img


# flask routes

@app.route('/')
def index():
    return """<!DOCTYPE html>
<html>
<head>
<title>UGV Navigation</title>
<style>
  * { box-sizing: border-box; }
  body { background: #0d1117; color: #eee; font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 10px; }
  h2 { margin: 5px 0; color: #58a6ff; font-size: 22px; }
  .top { display: flex; align-items: center; gap: 12px; margin-bottom: 8px; }
  #status { font-size: 16px; color: #3fb950; font-weight: bold; }
  .btn { color: white; border: none; padding: 8px 18px; border-radius: 6px; cursor: pointer; font-size: 14px; font-weight: bold; }
  .btn-stop { background: #da3633; }
  .btn-explore { background: #1f6feb; }
  .main { display: flex; gap: 10px; }
  .panel { background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 8px; }
  .panel b { color: #8b949e; font-size: 13px; }
  .map-container { position: relative; overflow: hidden; border: 1px solid #58a6ff; cursor: crosshair; }
  #map-img { display: block; transform-origin: 0 0; }
  #cam-img { display: block; width: 100%; border: 1px solid #30363d; }
  .cam-panel { width: 400px; flex-shrink: 0; }
  .zoom-controls { display: flex; gap: 4px; margin-top: 4px; }
  .zoom-btn { background: #30363d; color: #eee; border: 1px solid #484f58; padding: 4px 12px; border-radius: 4px; cursor: pointer; font-size: 16px; }
</style>
</head>
<body>
<h2>UGV Navigation Control</h2>
<div class="top">
  <span id="status">idle</span>
  <button class="btn btn-stop" onclick="stopRobot()">STOP</button>
  <button class="btn btn-explore" onclick="explore()">AUTO EXPLORE</button>
</div>
<div class="main">
  <div class="panel" style="flex:1">
    <b>MAP</b> &mdash; click to send robot, scroll to zoom, drag to pan
    <br>
    <div class="map-container" id="map-container">
      <img id="map-img" src="/map.png">
    </div>
    <div class="zoom-controls">
      <button class="zoom-btn" onclick="zoomMap(1.3)">+</button>
      <button class="zoom-btn" onclick="zoomMap(1/1.3)">-</button>
      <button class="zoom-btn" onclick="resetZoom()">Reset</button>
      <button class="zoom-btn" onclick="centerOnRobot()">Center Robot</button>
    </div>
    <div class="info" id="map-info" style="color:#8b949e;font-size:12px;margin-top:4px;">Click on map to navigate</div>
  </div>
  <div class="panel cam-panel">
    <b>FRONT CAMERA</b>
    <br>
    <img id="cam-img" src="/camera_stream">
    <br><br>
    <b>CHASE CAM</b>
    <br>
    <img id="overhead-img" src="/overhead_stream" style="width:100%">
  </div>
</div>
<script>
let zoom = 1, panX = 0, panY = 0;
const mapImg = document.getElementById('map-img');
const container = document.getElementById('map-container');

function applyTransform() {
  mapImg.style.transform = `translate(${panX}px, ${panY}px) scale(${zoom})`;
}

container.addEventListener('click', (e) => {
  const rect = mapImg.getBoundingClientRect();
  const px = (e.clientX - rect.left) / rect.width;
  const py = (e.clientY - rect.top) / rect.height;
  fetch('/goal?px=' + px + '&py=' + py)
    .then(r => r.text())
    .then(t => { document.getElementById('map-info').innerText = t; });
});

function zoomMap(factor) { zoom = Math.max(0.5, Math.min(5, zoom * factor)); applyTransform(); }
function resetZoom() { zoom = 1; panX = 0; panY = 0; applyTransform(); }
function centerOnRobot() { resetZoom(); zoom = 2; panX = -400*zoom+container.clientWidth/2; panY = -400*zoom+container.clientHeight/2; applyTransform(); }

setInterval(() => {
  const img = new window.Image();
  img.onload = () => { mapImg.src = img.src; };
  img.src = '/map.png?' + Date.now();
  fetch('/status').then(r=>r.text()).then(t => {
    document.getElementById('status').innerText = t;
    document.getElementById('status').style.color = t.includes('navigat') ? '#d29922' : t.includes('arrived') ? '#3fb950' : '#8b949e';
  });
}, 800);

function stopRobot() { fetch('/stop').then(r=>r.text()).then(t => { document.getElementById('status').innerText = t; }); }
function explore() { fetch('/explore').then(r=>r.text()).then(t => { document.getElementById('status').innerText = t; }); }
</script>
</body>
</html>"""


@app.route('/map.png')
def map_png():
    img = render_map_image()
    buf = io.BytesIO()
    img.save(buf, format='PNG', optimize=True)
    buf.seek(0)
    return send_file(buf, mimetype='image/png')


@app.route('/camera_stream')
def camera_stream():
    def gen():
        while True:
            with lock:
                frame = camera_jpeg
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.25)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/overhead_stream')
def overhead_stream():
    def gen():
        while True:
            with lock:
                frame = overhead_jpeg
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.25)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/clear_trail')
def clear_trail():
    global robot_trail
    with lock:
        robot_trail = []
    return 'trail cleared'


@app.route('/status')
def status():
    return nav_status

@app.route('/debug')
def debug():
    with lock:
        rw = robot_world
        rp = robot_pos
        tr = len(robot_trail)
    return f"world={rw} map={rp} trail_len={tr}"


@app.route('/goal')
def goal():
    px = float(request.args.get('px', 0.5))
    py = float(request.args.get('py', 0.5))
    wx = -WORLD_SIZE_X/2 + px * WORLD_SIZE_X
    wy = WORLD_SIZE_Y/2 - py * WORLD_SIZE_Y
    node.send_goal(wx, wy)
    return f"Goal: ({wx:.1f}, {wy:.1f})"


@app.route('/stop')
def stop():
    node.stop()
    return "stopped"


@app.route('/teleport')
def teleport():
    """Teleport robot and recalibrate odom offset"""
    global robot_world, robot_trail
    wx = float(request.args.get('x', -170))
    wy = float(request.args.get('y', 0))
    # teleport via gz
    import subprocess as sp
    GZ = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"
    sp.run([GZ, "service", "-s", "/world/outdoor_terrain/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--req", f'name: "husky_a200" position {{ x: {wx} y: {wy} z: 12.0 }} '
                     f'orientation {{ x: 0 y: 0 z: 0 w: 1 }}',
            "--timeout", "5000"], capture_output=True, timeout=10)
    time.sleep(3)
    # recalibrate odom
    node.spawn_offset_recalib(wx, wy)
    with lock:
        robot_world = (wx, wy, 0.0)
        robot_trail.clear()
    return f"Teleported to ({wx}, {wy})"


@app.route('/explore')
def explore():
    global nav_status
    nav_status = "exploring..."
    def do_explore():
        global nav_status
        # explore: forest -> open -> village
        waypoints = [(-30, 0), (-20, 0), (-10, 0), (0, 0), (10, 0), (20, 0), (30, 0)]
        for wx, wy in waypoints:
            if nav_status == "stopped":
                break
            node.send_goal(wx, wy)
            time.sleep(20)
        nav_status = "explore done"
    threading.Thread(target=do_explore, daemon=True).start()
    return "exploring..."


def ros_spin():
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    rclpy.init()
    node = NavWebNode()
    print("Waiting for Nav2...")
    node.nav_client.wait_for_server(timeout_sec=10.0)
    print("Nav2 ready!")

    # file-based pos tracking (drive_route.py writes /tmp/robot_pos.txt)
    def file_pos_thread():
        global robot_world, robot_trail
        while True:
            try:
                with open('/tmp/robot_pos.txt', 'r') as f:
                    parts = f.read().strip().split()
                if len(parts) >= 3:
                    wx, wy, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                    with lock:
                        robot_world = (wx, wy, yaw)
                        if not robot_trail or ((wx - robot_trail[-1][0])**2 + (wy - robot_trail[-1][1])**2) > 0.25:
                            robot_trail.append((wx, wy))
                            if len(robot_trail) > MAX_TRAIL:
                                robot_trail = robot_trail[-MAX_TRAIL:]
            except:
                pass
            time.sleep(0.3)

    threading.Thread(target=file_pos_thread, daemon=True).start()
    print("File-based position tracking started (/tmp/robot_pos.txt)")

    # ROS spin thread
    t = threading.Thread(target=ros_spin, daemon=True)
    t.start()

    print(f"Web UI: http://localhost:8765")
    app.run(host='0.0.0.0', port=8765, threaded=True)
