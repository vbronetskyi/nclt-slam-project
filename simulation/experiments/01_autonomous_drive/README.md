# Experiment 01: Route 1 — Forest to Road to Village

## Overview

Autonomous drive of Husky A200 UGV through a 390x390m outdoor terrain with 541 objects (trees, buildings, obstacles). The robot drives from the forest (west) through a dirt road to the village (east).

**Route 1: Forest → Road → Village**
- Start: (-170, 0) — dense forest
- Goal: (100, 0) — village with houses
- Path: gradually rises through forest (y=0→25), follows dirt road, descends to village
- Distance: 271m, 54 waypoints every 5m

## Latest Drive Results (GT-verified)

| Metric | Value |
|--------|-------|
| Status | **SUCCESS** |
| Duration | 667s (11.1 min) |
| Distance traveled | 271m |
| Final distance to goal | 2.4m |
| Average speed | 0.41 m/s |
| GT trajectory points | 33,453 @ 50Hz |
| GT accuracy | <1cm (verified by teleport test) |
| Collisions | 0 (this run) |

### Ground Truth System

Position tracking uses **Gazebo `dynamic_pose/info`** — publishes only moving models at 50Hz. This gives true world-frame position with <1cm accuracy (verified: teleport to 4 known positions, max error 0.01m).

Previous runs used odom+offset which drifted 10-20m over the route. GT eliminated this completely.

### Waypoint Passage (all 54 passed)

Forest section (WP0-WP30): robot navigates between trees at y=0→25
Road section (WP30-WP38): follows dirt road from (-20,25) to (20,10)
Village approach (WP38-WP54): descends to village at (100,0)

## Simulation Setup

| Component | Details |
|-----------|---------|
| Platform | Gazebo Harmonic (headless) + ROS 2 Jazzy, Ubuntu 24.04 |
| Robot | Husky A200 — 50kg, 4WD skid-steer, 0.67m wide, 0.33m wheels |
| Terrain | 390x390m, 6m elevation range, procedural grass/dirt texture |
| Objects | 541 models (427 trees, 4 houses, rocks, debris, barrels, cones, lamps) |
| Sensors | 2D LiDAR (360deg, 12m), RGBD camera (640x480), IMU (250Hz) |
| Cameras | Front camera (robot-mounted), Chase cam (3rd person behind robot) |
| SLAM | Disabled (slam_toolbox removed to save CPU/RAM) |
| GT | `/ground_truth/dynamic_poses` via PosePublisher plugin (50Hz) |
| Web UI | `web_nav.py` on port 8765 — map, cameras, click-to-drive |

### Robot Sensors (available ROS 2 topics)

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/odom` | Odometry | 50Hz | Skid-steer odom (drifts) |
| `/scan` | LaserScan | 10Hz | 360deg, 12m range |
| `/camera/color/image_raw` | Image | 30Hz | 640x480 RGB |
| `/camera/depth/image_rect_raw` | Image | 30Hz | 640x480 depth |
| `/camera/camera_info` | CameraInfo | 30Hz | Intrinsics (fx=554, fy=554, cx=320, cy=240) |
| `/imu/data` | Imu | 250Hz | 6-axis (accel + gyro) |
| `/camera/depth/color/points` | PointCloud2 | 10Hz | RGB-D point cloud |
| `/ground_truth/dynamic_poses` | TFMessage | 50Hz | GT world position |
| `/chase_camera/image` | Image | 15Hz | 3rd person view |

### Terrain Details

- Heightmap: 513x513 pixels, 6m Z range
- Visual mesh: 257x257 vertices with normals (10.6MB OBJ)
- Collision mesh: 257x257 (same resolution as visual — no clipping)
- Texture: 2048x2048, procedural (grass in open areas, leaf litter under trees, dirt road visible)
- Route corridors smoothed to max 7.5% slope

### Navigation Method

**Web-based click-to-drive + automated dense waypoint following:**
- 54 waypoints every 5m along pre-planned route
- P-controller: proportional steering toward next waypoint
- Acceptance radius: 3m
- Timeout: 15s per waypoint (skip on timeout)
- No runtime obstacle avoidance — relies on pre-planned clearance (2m+ from trees)
- GT position used for navigation decisions (no odom drift issues)

## Files

| File | Description |
|------|-------------|
| `gt_trajectory.csv` | Ground truth trajectory, 33,453 points @ 50Hz. Columns: `time,x,y,yaw` |
| `trajectory_r1.csv` | Earlier run with odom+IMU (5,232 points). Columns: `time,wx,wy,yaw,pitch,roll,imu_ax,imu_ay,imu_az` |
| `orb_slam3_results.md` | ORB-SLAM3 evaluation results and analysis |
| `trajectory_comparison.png` | Visual comparison of GT vs ORB-SLAM3 trajectory |
| `/workspace/simulation/routes/routes.json` | Route definition (11 coarse waypoints + road) |
| `/workspace/simulation/tools/drive_route.py` | Automated drive script with rosbag recording |
| `/workspace/simulation/tools/web_nav.py` | Web UI with GT tracking, click-to-drive |

## Known Issues & Solutions Found

| Issue | Status | Solution |
|-------|--------|----------|
| Odom drift (10-20m over 270m) | **Fixed** | GT from `dynamic_pose/info` (50Hz, <1cm) |
| Robot hitting trees | **Mitigated** | Dense 5m waypoints + 2m clearance from trees |
| Web map showing wrong position | **Fixed** | GT-only position writing (no odom conflict) |
| Gazebo OOM with 1000+ models | **Fixed** | Reduced to 541 models, disabled slam_toolbox |
| Camera lag during drive | **Fixed** | Disabled slam_toolbox LiDAR processing |
| Trail artifacts on teleport | **Fixed** | Auto-clear trail on >20m position jump |
| Terrain texture clipping | **Fixed** | Collision mesh same resolution as visual mesh |
| `robot_pos.txt` empty file race | **Fixed** | Atomic write (tmp + rename) |
| `TF_OLD_DATA` position jumps | **Fixed** | Fresh Gazebo restart clears TF buffer |

## Next Steps: 3D Mapping & Navigation

### Goal

Build a 3D map from this route for:
1. **Path planning** — navigate by pre-built map, not blind waypoints
2. **Visual SLAM localization** — know position from camera, not just odometry
3. **Autonomous re-navigation** — re-drive route or explore using the map

### Option A: RTAB-Map Live (recommended)

Run RTAB-Map during the drive instead of slam_toolbox. Processes RGB-D + IMU + odom in real-time.

**After one drive we get `rtabmap.db` containing:**
- Dense 3D point cloud (from depth camera)
- 2D occupancy grid (for Nav2 path planning)
- Visual vocabulary (for relocalization)
- Optimized pose graph (trajectory)
- Compressed keyframe images + depth

**Pipeline:**
```
1. Launch with slam_type:=rtabmap (config exists: rtabmap_params.yaml)
2. Drive Route 1 (11 min)
3. Save rtabmap.db (~50-200MB)
4. Export 2D map → .pgm/.yaml → Nav2 loads it
5. Next drive: RTAB-Map localization mode (no re-mapping)
6. Nav2 plans paths on the built map
```

**Pros:** One drive = complete map, Nav2-ready, compact storage, handles loop closures
**Cons:** Heavier CPU/RAM (depth processing), may need reduced camera rate

**Config:** `/workspace/simulation/src/ugv_navigation/config/rtabmap_params.yaml`
**Launch:** `full_sim.launch.py` accepts `slam_type:=rtabmap`

### Option B: ORB-SLAM3

Standalone visual-inertial SLAM. Needs camera frames + IMU.

**What we get:** Sparse 3D feature map + accurate trajectory
**What we don't get:** No dense point cloud, no 2D grid, no Nav2 integration

**To feed data:** Record rosbag during drive, replay through ORB-SLAM3 offline. Or run live with ROS 2 wrapper.

**Pros:** Lightweight, good accuracy, works with monocular camera
**Cons:** No Nav2 maps, needs octomap for grid, separate ROS 2 wrapper

### Option C: Hybrid

RTAB-Map for mapping (3D map + 2D grid) → ORB-SLAM3 for fast relocalization on subsequent drives.

### Comparison

| | RTAB-Map Live | ORB-SLAM3 Live | Rosbag + offline |
|---|---|---|---|
| Storage | ~100MB (.db) | ~50MB (map) | ~1-2GB (compressed) |
| Post-processing | None | Minimal | Full replay needed |
| Nav2 ready | Yes | No | After processing |
| 3D point cloud | Dense | Sparse | Depends |
| 2D occupancy grid | Yes | No | After processing |
| Relocalization | Yes | Yes | N/A |
| CPU during drive | High | Medium | Low (record only) |

### Recommendation

**Start with RTAB-Map Live (Option A):**
1. Drive Route 1 with `slam_type:=rtabmap` → get `rtabmap.db`
2. Export 2D map for Nav2 path planning
3. Test autonomous navigation on built map
4. If ORB-SLAM3 needed → record rosbag during same drive for offline replay

## Remaining Tasks

- [ ] Drive Route 1 with RTAB-Map (`slam_type:=rtabmap`) and save `.db`
- [ ] Export 2D map (`.pgm/.yaml`) from RTAB-Map
- [ ] Test Nav2 autonomous navigation on the built map
- [ ] Drive Route 2 (SW → Village) and Route 3 (NW → Village)
- [ ] (Optional) Record rosbag for ORB-SLAM3 offline processing
- [ ] (Optional) Set up ORB-SLAM3 for comparison
