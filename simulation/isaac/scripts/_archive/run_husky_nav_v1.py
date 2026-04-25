#!/usr/bin/env python3
"""Husky A200 navigation with obstacle avoidance in Isaac Sim
Uses ORB-SLAM3 map + depth-based obstacle detection + local planner.

usage:
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  /opt/isaac-sim-6.0.0/python.sh run_husky_nav.py --route road
"""
import os, sys, math, json, time, argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--route", type=str, default="road", choices=['road', "north", "south"])
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({
    "headless": True,
    "max_bounces": 4,
    "max_specular_transmission_bounces": 1,
    "max_volume_bounces": 1,
    "samples_per_pixel_per_frame": 48,
    "width": 640,
    "height": 480,
})

import omni, omni.kit.app, omni.kit.commands
import omni.graph.core as og
import usdrt.Sdf
import carb
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf, Usd

settings = carb.settings.get_settings()
settings.set("/rtx/raytracing/backfaceCulling", False)
settings.set("/rtx/directLighting/backfaceCulling", False)
settings.set("/rtx/post/motionblur/enabled", False)
settings.set("/rtx/post/dof/enabled", False)
settings.set("/rtx/post/bloom/enabled", False)
settings.set("/rtx/post/lensFlares/enabled", False)
settings.set("/rtx/directLighting/sampledLighting/enabled", False)
settings.set("/rtx/reflections/enabled", False)
settings.set("/rtx/indirectDiffuse/enabled", True)
settings.set("/persistent/omnigraph/updateToUsd", True)
settings.set("/persistent/omnihydra/useSceneGraphInstancing", False)

HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
SCENE_USD = "/opt/husky_forest_scene.usd"

# load scene
print("loading scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()

# load robot
print("adding Husky A200...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(300):
    app.update()

BASE_LINK = "/World/Husky/Geometry/base_link"

# physics 200Hz TGS
_phys = stage.GetPrimAtPath("/World/PhysicsScene")
if _phys.IsValid():
    PhysxSchema.PhysxSceneAPI(_phys).GetTimeStepsPerSecondAttr().Set(200)
    PhysxSchema.PhysxSceneAPI(_phys).CreateSolverTypeAttr().Set("TGS")

# wheel friction
from pxr import UsdShade
_wf = UsdShade.Material.Define(stage, "/World/WheelFriction")
_wfp = UsdPhysics.MaterialAPI.Apply(_wf.GetPrim())
_wfp.CreateStaticFrictionAttr(1.0)
_wfp.CreateDynamicFrictionAttr(0.8)
_wfp.CreateRestitutionAttr(0.0)
for wl in ["front_left_wheel_link", "front_right_wheel_link",
           "rear_left_wheel_link", "rear_right_wheel_link"]:
    col = stage.GetPrimAtPath(f"{BASE_LINK}/{wl}/collision")
    if col.IsValid():
        UsdShade.MaterialBindingAPI.Apply(col).Bind(_wf, materialPurpose="physics")

# camera
CAM_PATH = "/World/HuskyCamera"
cam = UsdGeom.Camera.Define(stage, CAM_PATH)
cam.CreateFocalLengthAttr(1.93)
cam.CreateHorizontalApertureAttr(3.86)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))
_cam_xf = UsdGeom.Xformable(cam)
_cam_op = _cam_xf.AddTransformOp()
# SPAWN_X = -95.0  # south route default, route 04-09 override this
CAM_FWD = 0.5
CAM_UP = 0.48

def _make_cam_matrix(x, y, z, yaw, pitch=0):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    r0x, r0y, r0z = sy, -cy, 0
    r1x, r1y, r1z = (-cy)*sp, (-sy)*sp, cp
    r2x, r2y, r2z = (-cy)*cp, (-sy)*cp, -sp
    return Gf.Matrix4d(r0x,r0y,r0z,0, r1x,r1y,r1z,0, r2x,r2y,r2z,0, x,y,z,1)

# spawn
husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_translate = husky_xf.AddTranslateOp()
_rotate = husky_xf.AddRotateXYZOp()

# terrain height
_RWPS = [
    (-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
    (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
    (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
    (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
    (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5),
]
def _road_y(x):
    if x <= _RWPS[0][0]: return _RWPS[0][1]
    if x >= _RWPS[-1][0]: return _RWPS[-1][1]
    for i in range(len(_RWPS)-1):
        if _RWPS[i][0] <= x <= _RWPS[i+1][0]:
            t = (x - _RWPS[i][0]) / (_RWPS[i+1][0] - _RWPS[i][0])
            return _RWPS[i][1] + t * (_RWPS[i+1][1] - _RWPS[i][1])
    return 0

def _terrain_height(x, y):
    h = 0.0
    h += 0.5 * math.sin(x*0.018+0.5) * math.cos(y*0.022+1.2)
    h += 0.35 * math.sin(x*0.035+2.1) * math.sin(y*0.03+0.7)
    h += 0.18 * math.sin(x*0.07+3.3) * math.cos(y*0.065+2.5)
    h += 0.12 * math.cos(x*0.11+1.0) * math.sin(y*0.09+4.0)
    h += 0.06 * math.sin(x*0.5+0.7) * math.cos(y*0.43+2.1)
    h += 0.04 * math.cos(x*0.7+3.5) * math.sin(y*0.6+0.4)
    h += 0.03 * math.sin(x*1.0+1.2) * math.cos(y*0.83+3.8)
    rd = abs(y - _road_y(x))
    if rd < 4.0: h *= (rd/4.0)**2
    if rd < 2.0: h -= 0.06*(1.0-rd/2.0)
    return max(h, -0.5)

# spawn position
_spawn_z = _terrain_height(-95, -6) + 0.5
_translate.Set(Gf.Vec3d(-95, -6, _spawn_z))
_rotate.Set(Gf.Vec3f(0, 0, 0))

# wheel drives
_wheel_vel_attrs = []
for wn in ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]:
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"/World/Husky/Physics/{wn}"), "angular")
    if drive:
        drive.GetDampingAttr().Set(100000.0)
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetMaxForceAttr().Set(500.0)
        _wheel_vel_attrs.append(drive.GetTargetVelocityAttr())

# start physics
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(300):
    app.update()

# camera render product
import omni.replicator.core as rep
rp = rep.create.render_product(CAM_PATH, (640, 480))
ann_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
ann_rgb.attach([rp])
ann_depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
ann_depth.attach([rp])
for _ in range(200):
    app.update()

# pose reading: ground truth from PhysX (used for camera sync + GT logging)
_base_prim = stage.GetPrimAtPath(BASE_LINK)
def get_gt_pose():
    # FIXME breaks if isaac restarts mid-run, works if clean start
    xf = UsdGeom.XformCache()
    tf = xf.GetLocalToWorldTransform(_base_prim)
    pos = tf.ExtractTranslation()
    rot = tf.ExtractRotationMatrix()
    yaw = math.atan2(rot[0][1], rot[0][0])
    return float(pos[0]), float(pos[1]), float(pos[2]), yaw

# SLAM pose reading (from rgbd_live process via /tmp/slam_pose.txt)
# the SLAM pose is in camera frame, need to convert to world frame   
# for navigation we need the initial transform (first SLAM pose = first GT pose)
_slam_origin = None      # (slam_x, slam_y, slam_z, gt_x, gt_y, gt_z, gt_yaw) at init
_slam_pose_file = "/tmp/slam_pose.txt"

def get_slam_pose():
    global _slam_origin
    try:
        with open(_slam_pose_file, "r") as f:
            line = f.readline().strip()
        parts = line.split()
        if len(parts) < 8:
            return None
        ts = float(parts[0])
        sx, sy, sz = float(parts[1]), float(parts[2]), float(parts[3])
        qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])

        # SLAM yaw from quaternion
        slam_yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        if _slam_origin is None:
            # first SLAM pose: record offset between SLAM frame and world frame
            gt = get_gt_pose()
            _slam_origin = (sx, sy, sz, slam_yaw, gt[0], gt[1], gt[2], gt[3])
            print(f"  SLAM origin: slam=({sx:.2f},{sy:.2f},{sz:.2f}) gt=({gt[0]:.1f},{gt[1]:.1f})")

        # transform SLAM pose to world frame using initial offset
        s0x, s0y, s0z, s0yaw, g0x, g0y, g0z, g0yaw = _slam_origin

        # delta in SLAM frame
        dx_s = sx - s0x
        dy_s = sy - s0y
        dz_s = sz - s0z
        dyaw = slam_yaw - s0yaw

        # SLAM frame: Z=forward, X=right(=-world Y), Y=down
        # rotate by initial heading to world frame
        cos_g = math.cos(g0yaw)
        sin_g = math.sin(g0yaw)

        # SLAM Z = forward along g0yaw, SLAM X = right of g0yaw
        wx = g0x + dz_s * cos_g + dx_s * sin_g
        wy = g0y + dz_s * sin_g - dx_s * cos_g
        wz = g0z - dy_s
        wyaw = g0yaw - dyaw  # SLAM yaw is inverted relative to world

        return wx, wy, wz, wyaw
    except (FileNotFoundError, ValueError, IndexError):
        return None

# navigation uses SLAM pose when available, falls back to GT   
def get_pose():
    slam = get_slam_pose()
    if slam is not None:
        return slam
    return get_gt_pose()

# start rgbd_live SLAM process
import subprocess
_slam_binary = "/workspace/third_party/ORB_SLAM3/Examples/RGB-D/rgbd_live"
_slam_vocab = "/workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
_slam_config = "/root/bags/husky_real/rgbd_d435i_v2.yaml"

# collision list (trees, rocks etc from scene)
_static_obstacles = []
with open("/tmp/gazebo_models.json") as f:
    models = json.load(f)
for m in models:
    if m["type"] in ("pine", "oak"):
        _static_obstacles.append((m["x"], m["y"], 0.7))
    elif m["type"] == "rock":
        _static_obstacles.append((m["x"], m["y"], 0.8))
    elif m["type"] == "house":
        _static_obstacles.append((m["x"], m["y"], 6.0))
    elif m["type"] in ("fallen_oak", "fallen_pine"):
        yaw = m.get("yaw", 0)
        for d in [-5, -3, -1, 0, 1, 3, 5]:
            _static_obstacles.append((m["x"]+d*math.cos(yaw), m["y"]+d*math.sin(yaw), 0.6))
    elif m["type"] == "shrub":
        _static_obstacles.append((m["x"], m["y"], 0.4))

#navigation: follow SLAM route waypoints + reactive depth dodge
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spawn_obstacles import spawn_obstacles, remove_obstacles

# load route from previous SLAM mapping run
with open("/tmp/slam_routes.json") as f:
    _all_routes = json.load(f)

LOOKAHEAD = 5.0
_path_idx = 0

def find_target(robot_x, robot_y, path):
    """pure pursuit with forward-only progress tracking"""
    global _path_idx
    if not path:
        return robot_x, robot_y

    search_end = min(_path_idx + 30, len(path))
    best_d = math.hypot(path[_path_idx][0]-robot_x, path[_path_idx][1]-robot_y)
    for i in range(_path_idx + 1, search_end):
        d = math.hypot(path[i][0]-robot_x, path[i][1]-robot_y)
        if d < best_d:
            best_d = d
            _path_idx = i

    remaining = LOOKAHEAD
    for i in range(_path_idx, len(path)-1):
        seg = math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
        if seg >= remaining:
            f = remaining / seg
            return path[i][0]+f*(path[i+1][0]-path[i][0]), path[i][1]+f*(path[i+1][1]-path[i][1])
        remaining -= seg
    return path[-1]


def steer(robot_yaw, tx, ty, rx, ry):
    yaw_err = math.atan2(ty-ry, tx-rx) - robot_yaw
    while yaw_err > math.pi: yaw_err -= 2*math.pi
    while yaw_err < -math.pi: yaw_err += 2*math.pi
    ang_v = max(-0.6, min(0.6, yaw_err * 1.0))
    if abs(yaw_err) < 0.2:
        lin_v = 1.0
    elif abs(yaw_err) < 0.5:
        lin_v = 0.6
    else:
        lin_v = 0.3
    return lin_v, ang_v


def check_path_blocked(d_img, target_bearing):
    """check if there's an obstacle in the direction we're heading.
    target_bearing: angle of target relative to robot heading (radians, + = left).
    only looks at upper-middle rows to ignore ground detections.
    returns (blocked, min_depth) or (False, 999)."""
    if d_img is None:
        return False, 999.0
    try:
        h, w = d_img.shape[:2]
        # D435i FOV +-87 deg = ±43.5 deg. map bearing to pixel column
        col = int(w / 2 - target_bearing * w / 1.52)
        col = max(0, min(w - 1, col))
        # vertical strip: rows h//4 to h//2 only (above horizon, no ground)
        margin = 40
        c0 = max(0, col - margin)
        c1 = min(w, col + margin)
        strip = d_img[h//4:h//2, c0:c1]
        valid = strip[(strip > 0.5) & (strip < 12.0)]
        if len(valid) < 30:
            return False, 999.0
        min_d = float(np.percentile(valid, 10))
        return min_d < 3.0, min_d
    except Exception:
        return False, 999.0


def plan_detour(rx, ry, ryaw, d_img):
    """plan a smooth detour around an obstacle.
    5 waypoints, max +-20 deg turn angle to keep SLAM tracking stable.
    returns list of (x, y) waypoints."""
    # which side is more open?
    go_left = True
    if d_img is not None:
        try:
            h, w = d_img.shape[:2]
            left = d_img[h//4:h//2, :w//3]
            right = d_img[h//4:h//2, 2*w//3:]
            lv = left[(left > 0.3) & (left < 15.0)]
            rv = right[(right > 0.3) & (right < 15.0)]
            left_clear = float(np.median(lv)) if len(lv) > 30 else 0
            right_clear = float(np.median(rv)) if len(rv) > 30 else 0
            go_left = left_clear >= right_clear
        except Exception:
            pass

    offset = 3.0  # max lateral offset
    fwd = math.cos(ryaw), math.sin(ryaw)
    if go_left:
        side = -math.sin(ryaw), math.cos(ryaw)
    else:
        side = math.sin(ryaw), -math.cos(ryaw)

    # 5 waypoints: gentle entry, full offset, maintain, gentle exit, rejoin
    detour = [
        (rx + 4*fwd[0] + 1.2*side[0],      ry + 4*fwd[1] + 1.2*side[1]),       # gentle start
        (rx + 8*fwd[0] + offset*side[0],    ry + 8*fwd[1] + offset*side[1]),    # full offset
        (rx + 12*fwd[0] + offset*side[0],   ry + 12*fwd[1] + offset*side[1]),   # maintain past obstacle
        (rx + 16*fwd[0] + 1.2*side[0],      ry + 16*fwd[1] + 1.2*side[1]),      # gentle return
        (rx + 20*fwd[0],                     ry + 20*fwd[1]),                     # back on route line
    ]
    side_name = "left" if go_left else "right"
    print(f"    DETOUR planned: {side_name}, {len(detour)} pts, offset={offset:.1f}m")
    return detour


# main scenario
print("\n=== NAVIGATION SCENARIO ===\n")

rec_dir = f"/root/bags/husky_real/isaac_nav_{int(time.time())}"
os.makedirs(f"{rec_dir}/camera_rgb", exist_ok=True)
os.makedirs(f"{rec_dir}/camera_depth", exist_ok=True)

#start SLAM
os.remove("/tmp/slam_stop") if os.path.exists("/tmp/slam_stop") else None
os.remove("/tmp/slam_pose.txt") if os.path.exists("/tmp/slam_pose.txt") else None
os.remove("/tmp/slam_status.txt") if os.path.exists("/tmp/slam_status.txt") else None
slam_proc = subprocess.Popen(
    [_slam_binary, _slam_vocab, _slam_config, rec_dir],
    stdout=open(f"{rec_dir}/slam_log.txt", "w"), stderr=subprocess.STDOUT)
print(f"  SLAM started (PID {slam_proc.pid})")
for _ in range(100):
    if os.path.exists("/tmp/slam_status.txt"): break
    time.sleep(0.1)

# spawn obstacles
print(f"  spawning obstacles on {args.route}...")
spawn_obstacles(stage, args.route)
for _ in range(30): app.update()

# build paths from SLAM route waypoints (road centerline from mapping run)
route_wps = [tuple(p) for p in _all_routes[args.route]]
# split into outbound (start->goal) and return (goal->start)
dists = [math.hypot(p[0]-72, p[1]+5) for p in route_wps]
turn_idx = dists.index(min(dists))
path_out = route_wps[:turn_idx+1]
path_back = route_wps[turn_idx+1:]
print(f"  route: {len(path_out)} out + {len(path_back)} back waypoints")

# recording files
gt_file = open(f"{rec_dir}/groundtruth.csv", "w")
gt_file.write("timestamp,x,y,z,yaw,phase\n")
nav_log = open(f"{rec_dir}/nav_log.csv", "w")
nav_log.write("timestamp,x,y,yaw,tx,ty,lin_v,ang_v,phase,path_len,slam_x,slam_y,dodge\n")

wheel_r = 0.165
track = 0.555
sim_time = 5.0
img_count = 0
from PIL import Image as PILImg


def drive_phase(current_path, dest_x, dest_y, phase_name):
    """follow route waypoints. when path is blocked, stop, plan detour, execute, rejoin."""
    global sim_time, img_count, _path_idx
    _path_idx = 0
    trajectory = []
    _last_depth = None
    _last_good_pose = None
    _detour = None        # None = normal mode, list = executing detour waypoints
    _detour_idx = 0
    _check_interval = 0   # counter for path-blocked check (~every 1s)

    print(f"\n--- {phase_name}: {len(current_path)} wps ---")

    while sim_time < 2000:
        app.update()
        sim_time += 1.0 / 60.0

        gt_x, gt_y, gt_z, gt_yaw = get_gt_pose()
        # SLAM for navigation. GT only until SLAM initializes
        slam = get_slam_pose()
        if slam is not None:
            sx, sy, sz, syaw = slam
            #sanity check: reject SLAM jumps > 5m from last good pose
            if _last_good_pose is not None:
                jump = math.hypot(sx - _last_good_pose[0], sy - _last_good_pose[1])
                if jump > 5.0:
                    # SLAM glitch - keep last good pose
                    rx, ry, rz, ryaw = _last_good_pose
                else:
                    rx, ry, rz, ryaw = sx, sy, sz, syaw
                    _last_good_pose = (rx, ry, rz, ryaw)
            else:
                rx, ry, rz, ryaw = sx, sy, sz, syaw
                _last_good_pose = (rx, ry, rz, ryaw)
        elif _slam_origin is None:
            rx, ry, rz, ryaw = gt_x, gt_y, gt_z, gt_yaw
            _last_good_pose = (rx, ry, rz, ryaw)
        else:
            for wa in _wheel_vel_attrs:
                wa.Set(0.0)
            continue

        if math.hypot(rx-dest_x, ry-dest_y) < 4.0:
            print(f"  ARRIVED at ({dest_x:.0f},{dest_y:.0f})")
            break

        frame_n = int(sim_time * 60)

        # read depth + save images every +-6 frames (+-10Hz)
        if frame_n % 6 == 0:
            _last_depth = ann_depth.get_data()
            rgb = ann_rgb.get_data()
            try:
                if rgb is not None and np.mean(rgb[:,:,:3]) > 1:
                    img = PILImg.fromarray(rgb[:,:,:3].astype(np.uint8)).resize((640, 480))
                    img.save(f"/tmp/isaac_cam_fwd.tmp.jpg", quality=75)
                    os.replace("/tmp/isaac_cam_fwd.tmp.jpg", "/tmp/isaac_cam_fwd.jpg")
                    img.save(f"{rec_dir}/camera_rgb/{sim_time:.4f}.jpg", quality=90)
                    if _last_depth is not None:
                        dc = np.nan_to_num(_last_depth, nan=0.0, posinf=0.0, neginf=0.0)
                        PILImg.fromarray((dc*1000).astype(np.uint16)).save(
                            f"{rec_dir}/camera_depth/{sim_time:.4f}.png")
                    img_count += 1
            except Exception:
                pass

        dodging = _detour is not None

        if _detour is not None:
            # detour mode: follow detour waypoints
            dtx, dty = _detour[_detour_idx]
            dist_to_dpt = math.hypot(dtx - rx, dty - ry)
            if dist_to_dpt < 2.5:
                _detour_idx += 1
                if _detour_idx >= len(_detour):
                    #detour complete - rejoin route
                    print(f"    DETOUR complete at ({rx:.0f},{ry:.0f})")
                    _detour = None
                    _detour_idx = 0
            if _detour is not None:
                dtx, dty = _detour[_detour_idx]
                tx, ty = dtx, dty
                lin_v, ang_v = steer(ryaw, tx, ty, rx, ry)
                lin_v = min(lin_v, 0.4)  # slow during detour for stable SLAM
        else:
            #normal mode: pure pursuit on route
            tx, ty = find_target(rx, ry, current_path)
            lin_v, ang_v = steer(ryaw, tx, ty, rx, ry)

            # check if path is blocked (every +-1s = every 60 frames)
            _check_interval += 1
            if _check_interval >= 60 and _last_depth is not None:
                _check_interval = 0
                target_bearing = math.atan2(ty - ry, tx - rx) - ryaw
                while target_bearing > math.pi: target_bearing -= 2*math.pi
                while target_bearing < -math.pi: target_bearing += 2*math.pi
                blocked, obs_dist = check_path_blocked(_last_depth, target_bearing)
                if blocked:
                    dist_to_target = math.hypot(tx - rx, ty - ry)
                    if obs_dist < dist_to_target:
                        # obstacle between us and target - stop and plan detour
                        print(f"    PATH BLOCKED at ({rx:.0f},{ry:.0f}), obs={obs_dist:.1f}m")
                        _detour = plan_detour(rx, ry, ryaw, _last_depth)
                        _detour_idx = 0
                        lin_v = 0.0
                        ang_v = 0.0

        v_left = (lin_v - ang_v * track / 2) / wheel_r
        v_right = (lin_v + ang_v * track / 2) / wheel_r
        for i, wa in enumerate(_wheel_vel_attrs):
            wa.Set(math.degrees(v_left if i % 2 == 0 else v_right))

        # camera follow
        cam_x = gt_x + CAM_FWD * math.cos(gt_yaw)
        cam_y = gt_y + CAM_FWD * math.sin(gt_yaw)
        cam_z = gt_z + CAM_UP
        fd = 0.5
        zf = _terrain_height(gt_x+fd*math.cos(gt_yaw), gt_y+fd*math.sin(gt_yaw))
        zb = _terrain_height(gt_x-fd*math.cos(gt_yaw), gt_y-fd*math.sin(gt_yaw))
        _cam_op.Set(_make_cam_matrix(cam_x, cam_y, cam_z, gt_yaw, math.atan2(zf-zb, 2*fd)))

        # log
        gt_file.write(f"{sim_time:.4f},{gt_x:.6f},{gt_y:.6f},{gt_z:.6f},{gt_yaw:.6f},{phase_name}\n")
        slam_str = f"{rx:.3f},{ry:.3f}" if slam is not None else "none,none"
        nav_log.write(f"{sim_time:.4f},{rx:.3f},{ry:.3f},{ryaw:.3f},{tx:.1f},{ty:.1f},{lin_v:.2f},{ang_v:.2f},{phase_name},{len(current_path)},{slam_str},{int(dodging)}\n")

        if frame_n % 10 == 0:
            try:
                with open("/tmp/isaac_robot_pos.txt", "w") as f:
                    f.write(f"{rx:.3f} {ry:.3f} {rz:.3f}")
            except: pass

        trajectory.append((gt_x, gt_y))
        if int(sim_time) % 20 == 0 and abs(sim_time - int(sim_time)) < 0.02:
            mode = " DETOUR" if dodging else ""
            print(f"  t={sim_time:.0f}s pos=({rx:.0f},{ry:.0f}) gt=({gt_x:.0f},{gt_y:.0f}){mode}")

    return trajectory


# phase 1: outbound with obstacles
traj_out = drive_phase(path_out, 72, -5, "outbound_obstacles")
print(f"\nphase 1 done: {len(traj_out)} poses, {img_count} images")

# remove obstacles
print("\nremoving obstacles...")
remove_obstacles(stage)
for _ in range(30): app.update()

# phase 2: return without obstacles
traj_back = drive_phase(path_back, -95, -6, "return_clear")
print(f"\nphase 2 done: {len(traj_back)} poses, {img_count} total images")

gt_file.close()
nav_log.close()
np.savez(f"{rec_dir}/trajectories.npz", outbound=np.array(traj_out), returning=np.array(traj_back))

print("\nstopping SLAM...")
with open("/tmp/slam_stop", "w") as f: f.write("stop")
try: slam_proc.wait(timeout=30)
except: pass

print(f"\n=== NAVIGATION COMPLETE ===")
print(f"  {rec_dir}")
print(f"  {img_count} images, {len(traj_out)}+{len(traj_back)} poses")

timeline.stop()
app.close()

#plot results
print("\ngenerating plots...")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(18, 5))

#road centerline
rx = np.linspace(-100, 78, 300)
ry = np.array([_road_y(x) for x in rx])
ax.plot(rx, ry, '--', color='gray', alpha=0.5, linewidth=1.5, label='Road')

# route waypoints
px_out = [p[0] for p in path_out]
py_out = [p[1] for p in path_out]
ax.plot(px_out, py_out, ':', color='blue', alpha=0.4, linewidth=1, label='Route (SLAM map)')

# actual trajectories
if traj_out:
    ox = [p[0] for p in traj_out[::3]]
    oy = [p[1] for p in traj_out[::3]]
    ax.plot(ox, oy, '-', color='red', linewidth=1.5, label='Outbound (obstacles)')
if traj_back:
    bx = [p[0] for p in traj_back[::3]]
    by = [p[1] for p in traj_back[::3]]
    ax.plot(bx, by, '-', color='green', linewidth=1.5, label='Return (clear)')

#obstacles: cones + tent
from spawn_obstacles import OBSTACLES
obs = OBSTACLES[args.route]
for group in obs["cones"]:
    for cx, cy in group:
        ax.plot(cx, cy, '^', color='orange', markersize=10, zorder=5)
ax.plot([], [], '^', color='orange', markersize=10, label='Cones')
tx, ty = obs["tent"]
ax.plot(tx, ty, 's', color='darkgreen', markersize=12, zorder=5, label='Tent')

# start / goal
ax.plot(-95, -6, 'ko', markersize=8, zorder=6)
ax.annotate('START', (-95, -6), textcoords="offset points", xytext=(5,8), fontsize=9, fontweight='bold')
ax.plot(72, -5, 'k*', markersize=12, zorder=6)
ax.annotate('GOAL', (72, -5), textcoords="offset points", xytext=(5,8), fontsize=9, fontweight='bold')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title(f'Navigation with obstacle avoidance - route: {args.route}')
ax.legend(loc='upper left', fontsize=8)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
fig.tight_layout()

plot_path = f"{rec_dir}/nav_trajectory.png"
fig.savefig(plot_path, dpi=150)
plt.close()
print(f"  saved: {plot_path}")

# also save to results/navigation for easy access
import shutil
result_dir = "/workspace/simulation/isaac/results/navigation"
os.makedirs(result_dir, exist_ok=True)
shutil.copy(plot_path, f"{result_dir}/08_nav_preplanned.png")
#and to final
shutil.copy(plot_path, "/workspace/simulation/isaac/results/final/08_nav_preplanned.png")
print(f"  copied to results/navigation/ and results/final/")
print("\ndone.")
