#!/usr/bin/env python3
"""Record drive data: front camera mp4, overhead mp4, GT trajectory

Run BEFORE driving, Ctrl+C when done

Outputs in /workspace/simulation/recordings/:
  - front_camera.mp4
  - overhead_camera.mp4
  - gt_trajectory.csv (copied from /tmp/gt_trajectory.csv)
  - trajectory_map.png (GT on terrain)
"""
import rclpy, threading, time, os, signal, sys, json, math, subprocess
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

OUT_DIR = '/workspace/simulation/recordings'
os.makedirs(OUT_DIR, exist_ok=True)

front_frames = []
overhead_frames = []
recording = True

class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.front_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.front_cb, qos)
        self.overhead_sub = self.create_subscription(
            Image, '/overhead_camera/image', self.overhead_cb, qos)
        self.front_count = 0
        self.overhead_count = 0

    def front_cb(self, msg):
        if not recording: return
        self.front_count += 1
        if self.front_count % 3 != 0: return
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            front_frames.append(img.copy())
        except: pass

    def overhead_cb(self, msg):
        if not recording: return
        self.overhead_count += 1
        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                if msg.encoding == 'bgr8':
                    img = img[:, :, ::-1]
                overhead_frames.append(img.copy())
        except: pass

def save_videos():
    for name, frames, fps in [('front_camera', front_frames, 10),
                                ('overhead_camera', overhead_frames, 10)]:
        if not frames:
            print(f"  {name}: no frames"); continue
        h, w = frames[0].shape[:2]
        path = f'{OUT_DIR}/{name}.mp4'
        proc = subprocess.Popen(
            ['ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
             '-s', f'{w}x{h}', '-pix_fmt', 'rgb24', '-r', str(fps),
             '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23', path],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for f in frames:
            proc.stdin.write(f.tobytes())
        proc.stdin.close()
        proc.wait()
        sz = os.path.getsize(path) / 1024 / 1024
        print(f"  {name}: {len(frames)} frames -> {sz:.1f}MB")

def save_trajectory_map():
    from PIL import Image as PILImage, ImageDraw
    gt_file = '/tmp/gt_trajectory.csv'
    if not os.path.exists(gt_file): return
    TEX = '/workspace/simulation/src/ugv_gazebo/worlds/terrain_texture.png'
    MAP_PX = 1000; HALF = 195.0
    tex = PILImage.open(TEX).resize((MAP_PX, MAP_PX))
    draw = ImageDraw.Draw(tex)
    def w2px(wx, wy):
        return int((wx+HALF)/390*MAP_PX), int((HALF-wy)/390*MAP_PX)
    rows = [l.strip().split(',') for l in open(gt_file) if l.strip()]
    traj = [(float(r[1]),float(r[2])) for r in rows if len(r)>=3]
    if not traj: return
    step = max(1, len(traj)//3000)
    tpx = [w2px(traj[i][0],traj[i][1]) for i in range(0,len(traj),step)]
    for i in range(1, len(tpx)):
        draw.line([tpx[i-1],tpx[i]], fill=(255,50,50), width=2)
    sx,sy = w2px(traj[0][0],traj[0][1])
    draw.ellipse([sx-8,sy-8,sx+8,sy+8], fill=(0,255,0), outline=(255,255,255), width=2)
    ex,ey = w2px(traj[-1][0],traj[-1][1])
    draw.ellipse([ex-8,ey-8,ex+8,ey+8], fill=(255,0,0), outline=(255,255,255), width=2)
    tex.save(f'{OUT_DIR}/trajectory_map.png')
    print(f"  trajectory_map.png saved")

def cleanup(signum=None, frame=None):
    global recording
    recording = False
    print("\n=== Saving ===")
    save_videos()
    gt = '/tmp/gt_trajectory.csv'
    if os.path.exists(gt):
        import shutil
        shutil.copy(gt, f'{OUT_DIR}/gt_trajectory.csv')
        print(f"  GT: {sum(1 for _ in open(gt))} points")
    save_trajectory_map()
    print("\n=== Files ===")
    for f in sorted(os.listdir(OUT_DIR)):
        p = os.path.join(OUT_DIR, f)
        if os.path.isfile(p):
            sz = os.path.getsize(p)
            print(f"  {f}: {sz/1024/1024:.1f}MB" if sz>1e6 else f"  {f}: {sz/1024:.0f}KB")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

print("=== Recorder started ===")
print(f"Output: {OUT_DIR}/")
print("Drive the robot. Ctrl+C to stop and save.\n")

rclpy.init()
node = Recorder()
try:
    while recording:
        rclpy.spin_once(node, timeout_sec=0.1)
except: pass
cleanup()
