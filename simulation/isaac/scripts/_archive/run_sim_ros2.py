#!/usr/bin/env python3   
"""run husky outdoor simulation with ros2 sensor publishing

  /camera/color/image_raw       sensor_msgs/Image (rgb8, 640x480, 30Hz)
  /camera/depth/image_rect_raw  sensor_msgs/Image (32FC1, 640x480, 30Hz)
  /camera/camera_info           sensor_msgs/CameraInfo
  /imu/data                     sensor_msgs/Imu (250Hz)
  /odom                         nav_msgs/Odometry (50Hz)
  /tf                           tf2_msgs/TFMessage

usage:
  /opt/isaac-sim-6.0.0/python.sh run_sim_ros2.py [--duration 60]
"""
import os
import sys
import argparse

# ros2 bridge needs these env vars BEFORE importing isaacsim
# the bundled jazzy libs must be on LD_LIBRARY_PATH
ISAAC_SIM_PATH = os.environ.get("ISAAC_SIM_PATH", '/opt/isaac-sim-6.0.0')
ROS2_LIB = os.path.join(ISAAC_SIM_PATH, "exts/isaacsim.ros2.core/jazzy/lib")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
if ROS2_LIB not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = ROS2_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", "")

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=60.0, help="sim duration in seconds")
args, _ = parser.parse_known_args()

# -- start isaac sim --
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni
import omni.kit.app
import omni.graph.core as og
import omni.kit.commands
import usdrt.Sdf
from pxr import Gf, UsdPhysics, PhysxSchema

SCENE_USD = "/workspace/simulation/isaac/assets/husky_outdoor_scene.usd"
ROBOT_PATH = "/husky"
BASE_LINK = f"{ROBOT_PATH}/Geometry/base_link"
CAM_LINK = f"{BASE_LINK}/top_plate_link/camera_realsense_bottom_screw_frame/camera_realsense_link"
CAM_PRIM = f"{CAM_LINK}/d435i_camera"
IMU_LINK = f"{BASE_LINK}/imu_link"

# enable ros2 extensions in dependency order
manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

# verify ros2 bridge loaded
eid = manager.get_enabled_extension_id("isaacsim.ros2.bridge")
if not eid:
    print("ERROR: ros2 bridge failed to load")
    print("  make sure LD_LIBRARY_PATH includes the jazzy libs:")
    print(f"  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{ROS2_LIB}")
    app.close()
    sys.exit(1)
print(f"ros2 bridge loaded: {eid}")

# -- load scene --
print(f"loading scene: {SCENE_USD}")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()

stage = omni.usd.get_context().get_stage()
print(f"  robot: {ROBOT_PATH}")

#-- create imu sensor prim if it doesn't have the right schema --
imu_sensor_path = f"{IMU_LINK}/imu_sensor"
imu_prim = stage.GetPrimAtPath(imu_sensor_path)
if not imu_prim.IsValid() or not imu_prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
    #recreate with proper IsaacSensorCreateImuSensor command
    try:
        if imu_prim.IsValid():
            stage.RemovePrim(imu_sensor_path)
        success, _ = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor",
            parent=IMU_LINK,
            sensor_period=1.0 / 250.0,  # 250 Hz
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )
        print(f"  imu sensor created: {imu_sensor_path} ({'ok' if success else 'failed'})")
    except Exception as e:
        print(f"  imu sensor creation error: {e}")

for _ in range(10):
    app.update()

# -- set up physics scene if needed --
phys_scene = stage.GetPrimAtPath("/World/PhysicsScene")
if phys_scene.IsValid():
    physx_api = PhysxSchema.PhysxSceneAPI.Apply(phys_scene)
    physx_api.CreateTimeStepsPerSecondAttr(60)
    print("  physics: 60 Hz")


# omnigraph: ros2 sensor publishing
print("\ncreating ros2 omnigraph...")

keys = og.Controller.Keys

(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ROS2SensorGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            # triggers
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),

            # camera render product
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),

            # camera publishers
            ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),

            # imu
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),

            # odometry
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),

            # tf tree
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            # -- camera render product --   
            ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAM_PRIM)]),
            ("CreateRenderProduct.inputs:width", 640),
            ("CreateRenderProduct.inputs:height", 480),

            # -- rgb camera --
            ("RGBPublish.inputs:topicName", "camera/color/image_raw"),
            ("RGBPublish.inputs:type", "rgb"),
            ("RGBPublish.inputs:frameId", "camera_realsense_link"),
            ("RGBPublish.inputs:resetSimulationTimeOnStop", True),

            # -- depth camera --
            ("DepthPublish.inputs:topicName", "camera/depth/image_rect_raw"),
            ("DepthPublish.inputs:type", "depth"),
            ("DepthPublish.inputs:frameId", "camera_realsense_link"),
            ("DepthPublish.inputs:resetSimulationTimeOnStop", True),

            # -- imu --
            ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_sensor_path)]),
            ("ReadIMU.inputs:readGravity", True),
            ("PublishIMU.inputs:topicName", "imu/data"),
            ("PublishIMU.inputs:frameId", "imu_link"),

            # -- odometry --
            ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(BASE_LINK)]),
            ("PublishOdom.inputs:topicName", "odom"),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_link"),

            # -- tf --
            ("PublishTF.inputs:topicName", "tf"),
            ("PublishTF.inputs:targetPrims", [
                usdrt.Sdf.Path(BASE_LINK),
                usdrt.Sdf.Path(CAM_LINK),
                usdrt.Sdf.Path(IMU_LINK),
            ]),
        ],
        keys.CONNECT: [
            # execution chain - ReadSimTime has no execIn, it auto-computes
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),

            # camera: render product -> publishers
            ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),

            # imu: read -> publish
            ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
            ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
            ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),

            #odometry: compute -> publish
            ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),

            # tf
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        ],
    },
)

print("  graph created: /ROS2SensorGraph")
print("  topics:")
print("    /camera/color/image_raw     (rgb8, 640x480)")
print("    /camera/depth/image_rect_raw (32FC1, 640x480)")
print("    /camera/camera_info")
print("    /imu/data                   (250Hz)")
print("    /odom                       (50Hz)")
print("    /tf")

# -- start simulation --
print(f"\nstarting simulation ({args.duration}s)...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# warm up renderer - RTX needs +-200 frames to produce non-black images
print("  warming up renderer...")
for _ in range(200):
    app.update()

print("simulation running, publishing ros2 topics")
print("  use 'ros2 topic list' in another terminal to verify")
print(f"  will run for {args.duration}s (ctrl+c to stop early)\n")

# main loop
physics_dt = 1.0 / 60.0
sim_time = 0.0
step = 0
status_interval = int(5.0 / physics_dt)

try:
    while sim_time < args.duration:
        app.update()
        sim_time += physics_dt
        step += 1

        if step % status_interval == 0:
            print(f"  t={sim_time:.1f}s / {args.duration:.0f}s")

except KeyboardInterrupt:
    print("\nstopped by user")

timeline.stop()
print(f"\nsimulation complete ({sim_time:.1f}s)")

app.close()
print("done")
