#!/usr/bin/env python3
"""
Husky A200 two-phase navigation with Nav2 in Isaac Sim.

Phase 1 (outbound): obstacles present, robot navigates start -> destination.
Phase 2 (return):   obstacles removed, robot navigates destination -> start.

Isaac Sim publishes sensors (RGB-D, odom, tf, clock) via ROS2 bridge.
Subscribes to /cmd_vel from Nav2 MPPI controller.
Localization: ORB-SLAM3 (slam_tf_publisher.py) or GT (--use-gt).

Phase transitions communicated via /tmp/nav2_phase.json:
  {"phase": 'outbound'|"removing"|"return"|"done", "timestamp": ...}

Records GT trajectory to results/navigation/ for analysis.

usage:
  # terminal 1: isaac sim
  /opt/isaac-sim-6.0.0/python.sh run_husky_nav2.py --route road --duration 900

  # terminal 2: slam tf + nav2 + goals
  bash start_nav2_all.sh
"""
import os, sys, math, json, time, argparse
import numpy as np

# ros2 env
ISAAC_SIM_PATH = os.environ.get("ISAAC_SIM_PATH", "/opt/isaac-sim-6.0.0")
ROS2_LIB = os.path.join(ISAAC_SIM_PATH, "exts/isaacsim.ros2.core/jazzy/lib")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
if ROS2_LIB not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = ROS2_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", "")

parser = argparse.ArgumentParser()
parser.add_argument("--route", type=str, default="road", choices=["road", "north", "south"])
parser.add_argument("--duration", type=float, default=900.0)
parser.add_argument("--no-obstacles", action="store_true", help="skip obstacle spawning")
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

# enable ros2 extensions
manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

eid = manager.get_enabled_extension_id("isaacsim.ros2.bridge")
if not eid:
    print("ERROR: ros2 bridge failed to load")
    app.close()
    sys.exit(1)
print(f"ros2 bridge: {eid}")

# load scene
HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
SCENE_USD = "/opt/husky_forest_scene.usd"

print("loading scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()

# print(f"DEBUG len(traj)={len(traj)}")
print("adding Husky A200...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(300):
    app.update()

BASE_LINK = "/World/Husky/Geometry/base_link"
CAM_LINK = f"{BASE_LINK}/top_plate_link/camera_realsense_bottom_screw_frame/camera_realsense_link"
IMU_LINK = f"{BASE_LINK}/imu_link"

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

# terrain helpers
def _road_y(x):
    RWPS = [(-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
            (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
            (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
            (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
            (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5)]
    if x <= RWPS[0][0]: return RWPS[0][1]
    if x >= RWPS[-1][0]: return RWPS[-1][1]
    for i in range(len(RWPS)-1):
        if RWPS[i][0] <= x <= RWPS[i+1][0]:
            t = (x - RWPS[i][0]) / (RWPS[i+1][0] - RWPS[i][0])
            return RWPS[i][1] + t * (RWPS[i+1][1] - RWPS[i][1])
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
husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_translate = husky_xf.AddTranslateOp()
_rotate = husky_xf.AddRotateXYZOp()
SPAWN_X, SPAWN_Y = -95.0, -6.0
DEST_X, DEST_Y = 72.0, -5.0
_spawn_z = _terrain_height(SPAWN_X, SPAWN_Y) + 0.5
_translate.Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, _spawn_z))
_rotate.Set(Gf.Vec3f(0, 0, 0))

# wheel drives
wheel_r = 0.165
track = 0.555
_wheel_vel_attrs = []
for wn in ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]:
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"/World/Husky/Physics/{wn}"), "angular")
    if drive:
        drive.GetDampingAttr().Set(100000.0)
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetMaxForceAttr().Set(500.0)
        _wheel_vel_attrs.append(drive.GetTargetVelocityAttr())

# ROS2 OmniGraph: sensor publishing + cmd_vel subscription
# (must be created BEFORE timeline.play() for ReadSimTime to work)
print("\ncreating ros2 omnigraph...")

CAM_PRIM = "/World/HuskyCamera"
cam = UsdGeom.Camera.Define(stage, CAM_PRIM)
cam.CreateFocalLengthAttr(1.93)
cam.CreateHorizontalApertureAttr(3.86)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))

keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ROS2NavGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            # PublishTF removed - always publishes to /tf with sim_time timestamps
            # regardless of topicName setting. TF published by tf_wall_clock_relay.py instead.
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("CamInfoPub", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        keys.SET_VALUES: [
            ("CreateRP.inputs:cameraPrim", [usdrt.Sdf.Path(CAM_PRIM)]),
            ("CreateRP.inputs:width", 640),
            ("CreateRP.inputs:height", 480),
            ("RGBPub.inputs:topicName", "camera/color/image_raw"),
            ("RGBPub.inputs:type", "rgb"),
            ("RGBPub.inputs:frameId", "camera_link"),
            ("DepthPub.inputs:topicName", "camera/depth/image_rect_raw"),
            ("DepthPub.inputs:type", "depth"),
            ("DepthPub.inputs:frameId", "camera_link"),
            ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(BASE_LINK)]),
            ("PublishOdom.inputs:topicName", "odom_isaac"),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_link"),
            # TF published by tf_wall_clock_relay.py using /odom_isaac data
            ("CamInfoPub.inputs:topicName", "camera/depth/camera_info"),
            ("CamInfoPub.inputs:frameId", "camera_link"),
            ("SubTwist.inputs:topicName", "cmd_vel"),
        ],
        # ReadSimTime output overridden with wall clock in the main loop.
        # use_sim_time=false everywhere - matches real robot behavior.
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CreateRP.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            # PublishTF removed from graph
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubTwist.inputs:execIn"),
            ("CreateRP.outputs:execOut", "RGBPub.inputs:execIn"),
            ("CreateRP.outputs:execOut", "DepthPub.inputs:execIn"),
            ("CreateRP.outputs:execOut", "CamInfoPub.inputs:execIn"),
            ("CreateRP.outputs:renderProductPath", "RGBPub.inputs:renderProductPath"),
            ("CreateRP.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
            ("CreateRP.outputs:renderProductPath", "CamInfoPub.inputs:renderProductPath"),
            ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
    },
)
print("  ros2 graph created")
print("  publishing: /camera/color/image_raw, /camera/depth/image_rect_raw, /odom, /tf, /clock")
print("  subscribing: /cmd_vel")

# start physics AFTER graph creation (ReadSimTime needs active timeline)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(300):
    app.update()

# warm up
for _ in range(200):
    app.update()

# spawn obstacles for outbound phase
obstacles_present = False
if not args.no_obstacles:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from spawn_obstacles import spawn_obstacles, remove_obstacles
    print(f"\nspawning obstacles on {args.route}...")
    spawn_obstacles(stage, args.route)
    obstacles_present = True
    for _ in range(30):
        app.update()

# phase management via /tmp/nav2_phase.json
PHASE_FILE = "/tmp/nav2_phase.json"

def write_phase(phase):
    with open(PHASE_FILE, 'w') as f:
        json.dump({"phase": phase, "timestamp": time.time(), "route": args.route}, f)
    print(f"  PHASE -> {phase}")

def read_phase():
    try:
        with open(PHASE_FILE) as f:
            return json.load(f).get("phase", "outbound")
    except (FileNotFoundError, json.JSONDecodeError):
        return "outbound"

write_phase("outbound")

# trajectory recording
RESULTS_DIR = "/workspace/simulation/isaac/results/navigation"
os.makedirs(RESULTS_DIR, exist_ok=True)
traj_file = os.path.join(RESULTS_DIR, f"nav2_trajectory_{args.route}_{int(time.time())}.csv")
traj_fp = open(traj_file, 'w')
traj_fp.write("time,phase,gt_x,gt_y,gt_z,gt_yaw,cmd_lin,cmd_ang\n")

# main loop
print(f"\n=== RUNNING (route={args.route}, duration={args.duration}s) ===")
print("  waiting for Nav2 cmd_vel on /cmd_vel topic...")
print(f"  trajectory recording: {traj_file}")
# print("DEBUG: isaac sim step")
print()

_base_prim = stage.GetPrimAtPath(BASE_LINK)
_cam_xf = UsdGeom.Xformable(cam)
_cam_op = _cam_xf.AddTransformOp()
CAM_FWD = 0.5
# ICP_THRESHOLD = 2.0  # was 2.0, 1.5 works on dense rooms but lost walls outdoors
CAM_UP = 0.48

def _make_cam_matrix(x, y, z, yaw, pitch=0):
    # TODO think about decomposing this monster function
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    r0x, r0y, r0z = sy, -cy, 0
    r1x, r1y, r1z = (-cy)*sp, (-sy)*sp, cp
    r2x, r2y, r2z = (-cy)*cp, (-sy)*cp, -sp
    return Gf.Matrix4d(r0x,r0y,r0z,0, r1x,r1y,r1z,0, r2x,r2y,r2z,0, x,y,z,1)

# get cmd_vel twist subscriber node
twist_node = None
for n in nodes:
    if "SubTwist" in str(og.Controller.node(n) if hasattr(n, '__str__') else n):
        twist_node = n
        break

sim_time = 0.0
physics_dt = 1.0 / 60.0
step = 0
current_phase = "outbound"

try:
    while sim_time < args.duration:
        app.update()
        sim_time += physics_dt
        step += 1

        # read cmd_vel from ROS2
        try:
            lin_x = og.Controller.attribute("outputs:linearVelocity", twist_node).get()[0]
            ang_z = og.Controller.attribute("outputs:angularVelocity", twist_node).get()[2]
        except Exception:
            lin_x = 0.0
            ang_z = 0.0

        # differential drive
        v_left = (lin_x - ang_z * track / 2) / wheel_r
        v_right = (lin_x + ang_z * track / 2) / wheel_r
        for i, wa in enumerate(_wheel_vel_attrs):
            wa.Set(math.degrees(v_left if i % 2 == 0 else v_right))

        # camera follow robot
        xf = UsdGeom.XformCache()
        tf = xf.GetLocalToWorldTransform(_base_prim)
        pos = tf.ExtractTranslation()
        rot = tf.ExtractRotationMatrix()
        gt_x, gt_y, gt_z = float(pos[0]), float(pos[1]), float(pos[2])
        gt_yaw = math.atan2(rot[0][1], rot[0][0])

        cam_x = gt_x + CAM_FWD * math.cos(gt_yaw)
        cam_y = gt_y + CAM_FWD * math.sin(gt_yaw)
        cam_z = gt_z + CAM_UP
        fd = 0.5
        zf = _terrain_height(gt_x+fd*math.cos(gt_yaw), gt_y+fd*math.sin(gt_yaw))
        zb = _terrain_height(gt_x-fd*math.cos(gt_yaw), gt_y-fd*math.sin(gt_yaw))
        _cam_op.Set(_make_cam_matrix(cam_x, cam_y, cam_z, gt_yaw, math.atan2(zf-zb, 2*fd)))

        # record trajectory (every 6th frame ~ 10 Hz)
        if step % 6 == 0:
            traj_fp.write(f"{sim_time:.3f},{current_phase},{gt_x:.3f},{gt_y:.3f},{gt_z:.3f},{gt_yaw:.4f},{lin_x:.3f},{ang_z:.3f}\n")

        # check phase transitions (every 60 frames ~ 1 Hz)
        if step % 60 == 0:
            new_phase = read_phase()
            if new_phase != current_phase:
                if new_phase == "removing" and obstacles_present:
                    print(f"\n  t={sim_time:.0f}s: REMOVING OBSTACLES")
                    remove_obstacles(stage)
                    obstacles_present = False
                    for _ in range(30):
                        app.update()
                    write_phase("return")
                    current_phase = "return"
                elif new_phase == "return":
                    current_phase = "return"
                elif new_phase == "done":
                    print(f"\n  t={sim_time:.0f}s: NAVIGATION COMPLETE")
                    break
                else:
                    current_phase = new_phase

        # telemetry every 10s
        if step % (60 * 10) == 0:
            dist_dest = math.hypot(gt_x - DEST_X, gt_y - DEST_Y)
            dist_start = math.hypot(gt_x - SPAWN_X, gt_y - SPAWN_Y)
            print(f"  t={sim_time:.0f}s phase={current_phase} pos=({gt_x:.1f},{gt_y:.1f}) "
                  f"cmd=({lin_x:.2f},{ang_z:.2f}) d_dest={dist_dest:.0f}m d_start={dist_start:.0f}m")

except KeyboardInterrupt:
    print("\nstopped by user")

traj_fp.close()
timeline.stop()
print(f"\nsimulation complete ({sim_time:.0f}s)")
print(f"trajectory saved: {traj_file}")
app.close()
