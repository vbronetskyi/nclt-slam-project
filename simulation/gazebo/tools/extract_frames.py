#!/usr/bin/env python3
"""Extract RGB, Depth, IMU from ROS2 topics to files for ORB-SLAM3
"""
import rclpy, numpy as np, cv2, time
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

OUT = '/workspace/simulation/orb_slam3_data'

def img_to_np(msg):
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    elif msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    return None

class Ext(Node):
    def __init__(self):
        super().__init__('ext')
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.dep_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 100)
        self.rf = open(f'{OUT}/rgb.txt', 'w')
        self.df = open(f'{OUT}/depth.txt', 'w')
        self.imf = open(f'{OUT}/imu.txt', 'w')
        self.af = open(f'{OUT}/associations.txt', 'w')
        self.rc = self.dc = self.ic = 0
        self.last_dt = None; self.last_dfn = None
        self.t0 = time.time()

    def rgb_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        img = img_to_np(msg)
        if img is None: return
        fn = f'rgb/{t:.6f}.png'
        cv2.imwrite(f'{OUT}/{fn}', img)
        self.rf.write(f'{t:.6f} {fn}\n')
        if self.last_dt:
            self.af.write(f'{t:.6f} {fn} {self.last_dt:.6f} {self.last_dfn}\n')
        self.rc += 1
        if self.rc % 500 == 0:
            print(f'  RGB:{self.rc} D:{self.dc} IMU:{self.ic} [{time.time()-self.t0:.0f}s]', flush=True)

    def dep_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d = img_to_np(msg)
        if d is None: return
        if d.dtype == np.float32: d = (d * 1000).astype(np.uint16)
        fn = f'depth/{t:.6f}.png'
        cv2.imwrite(f'{OUT}/{fn}', d)
        self.df.write(f'{t:.6f} {fn}\n')
        self.last_dt = t; self.last_dfn = fn; self.dc += 1

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g = msg.angular_velocity; a = msg.linear_acceleration
        self.imf.write(f'{t:.6f} {g.x:.6f} {g.y:.6f} {g.z:.6f} {a.x:.6f} {a.y:.6f} {a.z:.6f}\n')
        self.ic += 1

rclpy.init()
n = Ext()
try:
    rclpy.spin(n)
except:
    pass
n.rf.close(); n.df.close(); n.imf.close(); n.af.close()
print(f'\nDone: RGB={n.rc} D={n.dc} IMU={n.ic}', flush=True)
n.destroy_node()
rclpy.shutdown()
