#!/usr/bin/env python3
"""test ros2 topic publishing from isaac sim
runs the simulation for a few seconds and checks topics from within the same process

usage: /opt/isaac-sim-6.0.0/python.sh test_ros2_topics.py
requires: export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
          export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
"""
import os
import sys
import time

ISAAC_SIM_PATH = os.environ.get('ISAAC_SIM_PATH', "/opt/isaac-sim-6.0.0")
ROS2_LIB = os.path.join(ISAAC_SIM_PATH, "exts/isaacsim.ros2.core/jazzy/lib")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
if ROS2_LIB not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = ROS2_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", "")

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

# enable ros2
manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

eid = manager.get_enabled_extension_id("isaacsim.ros2.bridge")
assert eid, "ros2 bridge not loaded"
print(f"ros2 bridge: {eid}")

# load scene
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()

stage = omni.usd.get_context().get_stage()

# create imu sensor
imu_sensor_path = f"{IMU_LINK}/imu_sensor"
imu_prim = stage.GetPrimAtPath(imu_sensor_path)
if imu_prim.IsValid():
    stage.RemovePrim(imu_sensor_path)
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/imu_sensor", parent=IMU_LINK,
    sensor_period=1.0 / 250.0,
    linear_acceleration_filter_size=10,
    angular_velocity_filter_size=10,
    orientation_filter_size=10,
)
for _ in range(10):
    app.update()

# set physics
phys_scene = stage.GetPrimAtPath("/World/PhysicsScene")
if phys_scene.IsValid():
    physx_api = PhysxSchema.PhysxSceneAPI.Apply(phys_scene)
    physx_api.CreateTimeStepsPerSecondAttr(60)

# create omnigraph
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ROS2SensorGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAM_PRIM)]),
            ("CreateRenderProduct.inputs:width", 640),
            ("CreateRenderProduct.inputs:height", 480),
            ("RGBPublish.inputs:topicName", "camera/color/image_raw"),
            ("RGBPublish.inputs:type", "rgb"),
            ("RGBPublish.inputs:frameId", "camera_realsense_link"),
            ("DepthPublish.inputs:topicName", "camera/depth/image_rect_raw"),
            ("DepthPublish.inputs:type", "depth"),
            ("DepthPublish.inputs:frameId", "camera_realsense_link"),
            ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_sensor_path)]),
            ("ReadIMU.inputs:readGravity", True),
            ("PublishIMU.inputs:topicName", "imu/data"),
            ("PublishIMU.inputs:frameId", "imu_link"),
            ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(BASE_LINK)]),
            ("PublishOdom.inputs:topicName", "odom"),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_link"),
            ("PublishTF.inputs:topicName", "tf"),
            ("PublishTF.inputs:targetPrims", [
                usdrt.Sdf.Path(BASE_LINK),
                usdrt.Sdf.Path(CAM_LINK),
                usdrt.Sdf.Path(IMU_LINK),
            ]),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
            ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
            ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
            ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
            ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
            ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        ],
    },
)
print("omnigraph created")

# start sim
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# warm up - RTX renderer needs many frames to produce actual pixels
# first frames are always black
print("warming up renderer (200 frames)...")
for _ in range(200):
    app.update()

#now check topics from within the same process using rclpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import numpy as np

rclpy.init()
checker = rclpy.create_node("topic_checker")

counts = {}
first_img = None

def make_cb(topic):
    def cb(msg):
        global first_img
        counts[topic] = counts.get(topic, 0) + 1
        if topic == "/camera/color/image_raw" and first_img is None:
            first_img = msg
    return cb

checker.create_subscription(Image, "/camera/color/image_raw", make_cb("/camera/color/image_raw"), 10)
checker.create_subscription(Image, "/camera/depth/image_rect_raw", make_cb("/camera/depth/image_rect_raw"), 10)
checker.create_subscription(Imu, "/imu/data", make_cb("/imu/data"), 10)
checker.create_subscription(Odometry, "/odom", make_cb("/odom"), 10)
checker.create_subscription(TFMessage, "/tf", make_cb("/tf"), 10)

# run for 5 seconds, interleaving sim steps and rclpy spins
print("measuring topic rates for 5 seconds...")
start = time.time()
while time.time() - start < 5.0:
    app.update()
    rclpy.spin_once(checker, timeout_sec=0.001)

elapsed = time.time() - start
print(f"\nresults ({elapsed:.1f}s):")
for topic in sorted(counts):
    hz = counts[topic] / elapsed
    print(f"  {topic}: {counts[topic]} msgs, {hz:.1f} Hz")

# save camera frame
if first_img is not None:
    img = np.frombuffer(first_img.data, dtype=np.uint8)
    try:
        channels = len(first_img.data) // (first_img.height * first_img.width)
        img = img.reshape(first_img.height, first_img.width, channels)
        # save as PPM (rgb)
        with open("/tmp/isaac_camera.ppm", "wb") as f:
            f.write(f"P6\n{first_img.width} {first_img.height}\n255\n".encode())
            f.write(img[:, :, :3].tobytes())
        print(f"\ncamera frame saved: /tmp/isaac_camera.ppm ({first_img.width}x{first_img.height}, {first_img.encoding})")
    except Exception as e:
        print(f"save error: {e}")
else:
    print("\nno camera frames received")

# check an IMU reading
imu_data = {}
def imu_detail_cb(msg):
    imu_data["lin_acc"] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
    imu_data["ang_vel"] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    imu_data["orient"] = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

checker.create_subscription(Imu, "/imu/data", imu_detail_cb, 1)
for _ in range(30):
    app.update()
    rclpy.spin_once(checker, timeout_sec=0.001)

if imu_data:
    print(f"\nimu sample:")
    print(f"  lin_acc: [{imu_data['lin_acc'][0]:.3f}, {imu_data['lin_acc'][1]:.3f}, {imu_data['lin_acc'][2]:.3f}] m/s²")
    print(f"  ang_vel: [{imu_data['ang_vel'][0]:.4f}, {imu_data['ang_vel'][1]:.4f}, {imu_data['ang_vel'][2]:.4f}] rad/s")
    # gravity check: z accel should be +-9.81 when stationary
    if abs(imu_data["lin_acc"][2] - 9.81) < 2.0:
        print("  gravity check: PASS (z accel ≈ 9.81)")
    else:
        print(f"  gravity check: WARN (z accel = {imu_data['lin_acc'][2]:.2f})")

checker.destroy_node()
rclpy.shutdown()

timeline.stop()
app.close()
print("\ndone")
