#!/usr/bin/env python3
"""
load NVIDIA Rivermark outdoor environment and spawn Husky A200 in it
captures forward camera + topdown views to evaluate visual quality

usage: /opt/isaac-sim-6.0.0/python.sh load_rivermark.py
requires: export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
          export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
"""
import os
import sys
import time
import numpy as np

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni
import omni.kit.app
import omni.kit.commands
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf, PhysicsSchemaTools

ASSETS_ROOT = 'https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0'
RIVERMARK_URL = f"{ASSETS_ROOT}/Isaac/Environments/Outdoor/Rivermark/rivermark.usd"
SKY_HDR_URL = f"{ASSETS_ROOT}/NVIDIA/Assets/Skies/2022_1/Skies/Clear/noon_grass.hdr"
HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"


# 1. CREATE SCENE
print("creating scene...")
omni.usd.get_context().new_stage()
for _ in range(10):
    app.update()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# physics
physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/PhysicsScene")).CreateTimeStepsPerSecondAttr(60)


# 2. LOAD RIVERMARK
print("\nloading Rivermark outdoor scene from NVIDIA S3...")
env_prim = stage.DefinePrim("/World/Environment", "Xform")
env_prim.GetReferences().AddReference(RIVERMARK_URL)

# HDR sky
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(1000)
dome.CreateTextureFileAttr(SKY_HDR_URL)
dome.CreateTextureFormatAttr("latlong")

# physics ground (rivermark has visual ground but no collision)
PhysicsSchemaTools.addGroundPlane(
    stage, "/World/GroundPlane", "Z", 500.0,
    Gf.Vec3f(0, 0, -0.01), Gf.Vec3f(0.3, 0.5, 0.2)
)

# let assets load from S3
for i in range(150):
    app.update()
    if i % 50 == 0:
        print(f"  loading... ({i}/150)")
print("  rivermark loaded")


# 3. SPAWN HUSKY
print("\nspawning Husky A200...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(20):
    app.update()

# remove root_joint
rj = stage.GetPrimAtPath("/World/Husky/Physics/root_joint")
if rj.IsValid():
    stage.RemovePrim("/World/Husky/Physics/root_joint")
    print("  removed root_joint")

# spawn position
base_link = stage.GetPrimAtPath("/World/Husky/Geometry/base_link")
if base_link.IsValid():
    xf = UsdGeom.Xformable(base_link)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))
    print("  spawn: (0, 0, 0.5)")


# 4. CREATE CAMERAS (standalone, not attached to robot to avoid render conflicts)
print("\ncreating cameras...")

# forward camera - at robot's approximate camera position
fwd_cam = UsdGeom.Camera.Define(stage, "/World/ForwardCamera")
fwd_cam.CreateFocalLengthAttr(18.0)
fwd_cam.CreateHorizontalApertureAttr(20.955)
fwd_cam.CreateClippingRangeAttr(Gf.Vec2f(0.3, 500.0))
fwd_xf = UsdGeom.Xformable(fwd_cam)
fwd_xf.AddTranslateOp().Set(Gf.Vec3d(0.3, 0, 1.0))  # ~camera height on husky
fwd_xf.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, -90))  # look +X, up +Z
print("  forward: /World/ForwardCamera (0.3, 0, 1.0) looking +X")

# topdown camera
top_cam = UsdGeom.Camera.Define(stage, "/World/TopDownCamera")
top_cam.CreateFocalLengthAttr(15.0)
top_cam.CreateHorizontalApertureAttr(36.0)
top_cam.CreateClippingRangeAttr(Gf.Vec2f(1.0, 500.0))
top_xf = UsdGeom.Xformable(top_cam)
top_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 80))
# print(f"DEBUG len(traj)={len(traj)}")
print("  topdown: /World/TopDownCamera (0, 0, 80) looking -Z")

for _ in range(20):
    app.update()


# 5. START SIMULATION + WARM UP RENDERER
print("\nstarting simulation...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("warming up RTX renderer (500 frames)...")
for i in range(500):
    app.update()
    if i % 100 == 0:
        print(f"  warmup: {i}/500")


# 6. CAPTURE FRAMES WITH Camera API
from isaacsim.sensors.camera import Camera
from PIL import Image

# capture forward view first (only one camera at a time to avoid render conflicts)
print("\ncapturing forward view...")
cam_fwd = Camera("/World/ForwardCamera", name="fwd", frequency=30, resolution=(640, 480))
cam_fwd.initialize()
# need enough frames for RTX to render this camera's view
for _ in range(120):
    app.update()

# try multiple times - first frames can be black while renderer initializes
for attempt in range(5):
    rgb_fwd = cam_fwd.get_rgb()
    if rgb_fwd is not None and np.mean(rgb_fwd) > 1:
        break
    for _ in range(30):
        app.update()

if rgb_fwd is not None and np.mean(rgb_fwd) > 1:
    Image.fromarray(rgb_fwd[:, :, :3]).save("/tmp/rivermark_forward.png")
    print(f"  saved /tmp/rivermark_forward.png ({rgb_fwd.shape[1]}x{rgb_fwd.shape[0]}, mean={np.mean(rgb_fwd):.0f})")
else:
    print(f"  forward: mean={np.mean(rgb_fwd) if rgb_fwd is not None else 'None'}")

# now capture topdown
print("\ncapturing topdown view...")
cam_top = Camera("/World/TopDownCamera", name="top", frequency=10, resolution=(800, 600))
cam_top.initialize()
for _ in range(120):
    app.update()

for attempt in range(5):
    rgb_top = cam_top.get_rgb()
    if rgb_top is not None and np.mean(rgb_top) > 1:
        break
    for _ in range(30):
        app.update()

if rgb_top is not None and np.mean(rgb_top) > 1:
    Image.fromarray(rgb_top[:, :, :3]).save("/tmp/rivermark_topdown.png")
    print(f"  saved /tmp/rivermark_topdown.png ({rgb_top.shape[1]}x{rgb_top.shape[0]}, mean={np.mean(rgb_top):.0f})")
else:
    # print(f"DEBUG len(traj)={len(traj)}")
    print(f"  topdown: mean={np.mean(rgb_top) if rgb_top is not None else 'None'}")


# 7. MEASURE RENDER RATE
print("\nmeasuring render rate (10s)...")
count = 0
start = time.time()
while time.time() - start < 10.0:
    app.update()
    rgb = cam_fwd.get_rgb()
    if rgb is not None and np.mean(rgb) > 1:
        count += 1

elapsed = time.time() - start
hz = count / elapsed if elapsed > 0 else 0

# robot position
xfc = UsdGeom.XformCache()
if base_link.IsValid():
    mat = xfc.GetLocalToWorldTransform(base_link)
    pos = mat.ExtractTranslation()
    print(f"robot position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
print(f"render rate: {count} frames / {elapsed:.1f}s = {hz:.1f} Hz")

# save scene
scene_out = "/workspace/simulation/isaac/assets/husky_rivermark_scene.usd"
stage.Export(scene_out)
print(f"\nscene saved: {scene_out}")

timeline.stop()
app.close()
print("done")
