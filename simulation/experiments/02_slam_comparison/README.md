# Experiment 02: SLAM Comparison -- RTAB-Map vs ORB-SLAM3 (220x150m World)

## Overview

Compact outdoor terrain for UGV SLAM testing. Robot drives from dense forest through a curved dirt road to a village with houses.

**World size:** 220m (east-west) x 150m (north-south)
**Route:** (-105, -8) → curved road → (82, -13) village
**Total models:** 370

## Map Layout

```
West (-110)          Center (0)           East (110)
  ┌──────────────────────────────────────────────┐
  │  DENSE FOREST     Open field    VILLAGE      │  North (+75)
  │  (200 trees)       (road        (5 houses    │
  │  oak + pine)        visible)     + ruins)    │
  │                                              │
  │  ──────── curved dirt road ──────────────►   │  y ≈ road_y(x)
  │  Robot                              Goal     │
  │  spawn                             (82,-13)  │
  │  (-105,-8)                                   │
  │                                              │  South (-75)
  └──────────────────────────────────────────────┘
```

## Terrain

| Parameter | Value |
|-----------|-------|
| Heightmap | 257x257 pixels, 16-bit PNG |
| Z range | 0-10m (visible hills and bumps) |
| Mean slope | 13% |
| Max slope | 51% |
| Route corridor slope | max 10% (smoothed) |
| Texture | 2048x2048, procedural (grass/dirt/road) |
| Visual mesh | 257x257 vertices, OBJ with normals |
| Collision mesh | 257x257, STL (matches visual exactly) |

### Terrain Features
- **Sharp bumps** at 1-2m scale for rough ground feel
- **Route corridor** smoothed within 10m of road center
- **Dirt patches** under tree canopy (brown-green variation in forest)
- **Dry grass** on elevated/exposed areas
- **Curved dirt road** with wheel ruts visible on texture

### Road Curve
Road follows bezier curve: `y = (1-t)^2 * (-8) + 2*(1-t)*t * 6 + t^2 * (-3)`
where `t = (x+105) / 180`, from (-105, -8) to (75, -3).

Road ends between houses at approximately (75, -3).

## Objects (370 total)

| Type | Count | Location | Notes |
|------|-------|----------|-------|
| Trees (standing) | 297 | Forest + field + village bg | Oak tree, Pine Tree (Fuel) |
| Fallen trees | 40 | Forest | Rotated 80-90° (pitch 1.3-1.57) |
| Rocks | 23 | Along route + village | Falling Rock 1 (Fuel) |
| Houses | 5 | Village (x=65..105) | Lake House (PBR materials) |
| Ruins | 1 | Road center (x=-5) | Collapsed House |
| Barrels | 4 | Village | Construction Barrel |

### Object Zones
- **Dense forest** (x=-110 to -10): ~230 trees, 3m min spacing, fallen trees
- **Transition** (x=-10 to 40): scattered trees, rocks along road
- **Village** (x=40 to 110): houses, debris, background trees
- **Road corridor**: 6m clearance from road center (no objects within 6m)

### House Models
- `Lake House` (Gazebo Fuel) — has PBR materials, renders correctly in ogre2
- `Collapsed House` — ruin model, works in headless mode
- Note: `House 1` and `House 2` models render BLACK in Gazebo Harmonic ogre2 headless (script-based materials not supported)

## Robot & Sensors

| Component | Details |
|-----------|---------|
| Robot | Husky A200, 50kg, 4WD skid-steer |
| Front camera | 640x480 @ 30Hz requested (~10-15Hz actual) |
| Depth camera | 640x480, synced with RGB |
| IMU | 250Hz (accel + gyro) |
| LiDAR | Disabled (GPU savings) |
| Chase camera | 160x90 @ 2Hz (web display only) |
| Max speed | 0.9 m/s |

### Camera Intrinsics
- fx = fy = 382.0 (with overhead camera) or 554.0 (without)
- cx = 320.0, cy = 240.0
- No distortion (simulated)

### Actual Frame Rates
| Configuration | Camera Hz |
|--------------|-----------|
| 764 models (old map) | 5-9 Hz |
| 370 models (this map) | 10-15 Hz |
| Without web_nav | +1-2 Hz |
| During rosbag recording | -2-3 Hz |

## Ground Truth

Position tracking via Gazebo `dynamic_pose/info` topic:
- **Accuracy:** <1cm (verified)
- **Rate:** 50Hz
- **Format:** `/tmp/gt_trajectory.csv` (time, x, y, yaw)
- **Fallback:** Odom + spawn offset (drifts over distance)

## Navigation

**Click-to-drive** via web UI (http://localhost:8765):
- Click on map → robot drives to point
- P-controller with proportional steering
- Speed: 0.9 m/s (straight), 0.1-0.5 m/s (turning)
- No obstacle avoidance — manual driving around trees

## Data Recording

### Rosbag (uncompressed)
```bash
ros2 bag record -o bags/route_1_clean \
  --topics /camera/color/image_raw /camera/depth/image_rect_raw \
  /camera/camera_info /imu/data /odom /tf /clock /chase_camera/image
```

### Recorded Data Quality
| Topic | Count (typical 5min drive) | Rate |
|-------|---------------------------|------|
| RGB frames | ~3000 | ~10 Hz |
| Depth frames | ~3000 | ~10 Hz (synced) |
| IMU | ~75000 | ~250 Hz |
| Odom | ~15000 | ~50 Hz |
| GT trajectory | ~15000 | ~50 Hz |

### Video vs Simulation Quality
- **Rosbag records actual rendered frames** — same quality as what camera sees
- **Frame drops during lag** — if Gazebo renders at 8Hz, rosbag gets 8Hz (no interpolation)
- **Lag does NOT corrupt data** — frames are timestamped, SLAM algorithms handle variable frame rate
- **IMU is NOT affected by render lag** — IMU runs at 250Hz regardless of camera performance
- **Recommendation:** Stop web_nav during recording to gain +2-3 Hz on camera

## Known Issues

| Issue | Status | Notes |
|-------|--------|-------|
| Camera 10Hz instead of 30Hz | **Expected** | GPU bottleneck with 370 models |
| House 1/2 render black | **Fixed** | Replaced with Lake House (PBR) |
| Robot under terrain | **Fixed** | Spawn Z corrected, Y matched to road |
| Odom drift | **Mitigated** | GT from dynamic_pose used instead |
| ORB-SLAM3 tracking loss | **Known** | Monotonic Gazebo textures, 174 re-inits |

## ORB-SLAM3 Results (previous map)

Tested on 390x390m map with 764 models:
- **ATE:** 35.4m RMSE (poor -- constant re-initialization)
- **Map points:** 32-616 per map (insufficient for stable tracking)
- **Root cause:** Low-texture Gazebo rendering, 9Hz camera
- See `../01_autonomous_drive/orb_slam3_results.md` for details

Expected improvement with this map:
- Fewer models -> higher camera Hz (~10-15 vs 5-9)
- Smaller world -> more features per frame
- Rocks/barrels near route -> geometric features for tracking

## SLAM Results

Detailed quantitative comparison is in [slam_results.md](slam_results.md).

## ORB-SLAM3 Configs

ORB-SLAM3 configuration files are in the `config/` subdirectory:
- `gazebo_d435i.yaml` -- camera calibration for simulated D435i
- `gazebo_aggressive.yaml` -- relaxed thresholds for low-texture environments

## Large Files

`rtabmap.db` (282 MB) is gitignored due to its size. To reproduce it, follow the steps in the Reproducibility section below -- run RTAB-Map live during a drive through the 220x150m world.

## Files

| File | Description |
|------|-------------|
| `slam_results.md` | Detailed SLAM comparison results and analysis |
| `gt_trajectory.csv` | Ground truth (108,327 points @ 50Hz) |
| `rtabmap.db` | RTAB-Map database (282 MB, gitignored) |
| `rtabmap_poses.txt` | RTAB-Map exported poses (TUM format) |
| `CameraTrajectory.txt` | ORB-SLAM3 output (505 poses) |
| `trajectory_comparison.png` | Visual comparison plot |
| `results.json` | Metrics in JSON format |
| `config/gazebo_d435i.yaml` | ORB-SLAM3 camera config |
| `config/gazebo_aggressive.yaml` | ORB-SLAM3 aggressive config |
| `scripts/` | Extraction and evaluation scripts (see Reproducibility) |

### Simulation World Files (in parent repo)

| File | Description |
|------|-------------|
| `../../worlds/outdoor_terrain.sdf` | World definition (370 models) |
| `../../worlds/heightmap.png` | 257x257 heightmap, 16-bit |
| `../../worlds/terrain_texture.png` | 2048x2048 procedural texture |
| `../../routes/routes.json` | Route waypoints |

## Reproducibility

This world was generated procedurally with `np.random.seed(42)`. The generation script is embedded in the conversation history but not saved as a standalone file.

To regenerate, the key parameters are:
- World: 220x150m
- Heightmap: 257x257, Z_SCALE=10, sharp bumps (sigma=0.8-1.5)
- Road: bezier curve (-105,-8)→(0,6)→(75,-3)
- Forest: 200+ trees, x=-110..-10, min spacing 3m
- Village: Lake House models at (65,-12), (85,6), (95,-18)
- Route clearance: 6m from road center

## Launch

```bash
# Build
cd /workspace/simulation
colcon build --symlink-install --packages-select ugv_gazebo ugv_description ugv_navigation

# Launch simulation
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch ugv_gazebo full_sim.launch.py headless:=true

# Web UI
python3 tools/web_nav.py
# Open http://localhost:8765

# Record data
ros2 bag record -o bags/route_1_clean \
  --topics /camera/color/image_raw /camera/depth/image_rect_raw \
  /camera/camera_info /imu/data /odom /tf /clock
```
