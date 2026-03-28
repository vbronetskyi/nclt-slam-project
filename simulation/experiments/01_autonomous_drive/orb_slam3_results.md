# ORB-SLAM3 Results — Route 1

## Setup

### Input Data
- **Rosbag**: `bags/route_1_clean/` (30GB, uncompressed)
- **RGB**: 14,713 frames @ ~13Hz (640x480, rgb8)
- **Depth**: 14,719 frames @ ~13Hz (640x480, 32FC1 → uint16 mm)
- **IMU**: 367,971 samples @ ~321Hz (Phidgets Spatial 1042 simulated)
- **Duration**: 1,146s (19 min)
- **Route**: (-170, 0) → (98, 2) through forest + dirt road + village

### Data Extraction
- Rosbag played at 2x speed through ROS 2 subscriber
- RGB saved as PNG (BGR), Depth as 16-bit PNG (mm), IMU as text
- Extracted: 10,293 RGB + 10,290 Depth + 413,454 IMU samples (after cleanup)
- Null byte corruption fixed (extraction process killed during write)
- Timestamp alignment: overlap 2,622s, 18,493 usable frames, 25 IMU/frame

### ORB-SLAM3 Configuration
```yaml
Camera: PinHole, 640x480
  fx=fy=382.0, cx=320.0, cy=240.0
  No distortion (simulated camera)
  FPS: 9 (effective after subsampling)
RGBD.DepthMapFactor: 1000.0 (depth in mm)
ORBextractor:
  nFeatures: 1500
  scaleFactor: 1.2
  nLevels: 8
  iniThFAST: 20
  minThFAST: 7
```

### ORB-SLAM3 Mode
- **RGB-D** (without IMU) — IMU initialization failed due to:
  - Robot moves slowly (0.5 m/s) with minimal acceleration
  - Gravity-dominated IMU signal, not enough excitation for bias estimation
  - ORB-SLAM3 requires "not enough acceleration" to initialize IMU
- Used `rgbd_tum` binary with viewer disabled, no realtime sleep

### Processing
- Every 5th frame used (3,699 frames from 18,493)
- Processing time: ~40s (11ms/frame mean tracking)
- Total maps created: 174 (constant re-initialization)

## Results

### Trajectory
- **Input frames**: 3,699
- **Tracked frames**: 3,035 (82%)
- **Matched to GT**: 967 poses (after time alignment)
- **Maps created**: 174 (constant tracking loss → re-init)

### Accuracy (after Umeyama alignment)
| Metric | Value |
|--------|-------|
| ATE RMSE | 41.76m |
| ATE Mean | 35.45m |
| ATE Median | 31.18m |
| ATE Max | 83.69m |
| Scale factor | 10.85 (should be ~1.0 for RGB-D) |

### Map Points
- Per map: 32-616 points (varies by map)
- Best map: 616 points
- Most maps: 30-100 points (insufficient for stable tracking)
- Total across all 174 maps: not consolidated

## Analysis

### Why 174 Maps?
ORB-SLAM3 creates a new map when tracking is lost. The cycle:
1. Initialize with 30-600 map points from nearby features
2. Track for 2-10 frames successfully
3. "Fail to track local map!" — not enough feature matches between frames
4. Reset and create new map
5. Repeat

### Why So Few Features?
Gazebo ogre2 renderer produces images with:
- **Uniform green terrain** — grass texture is smooth, low contrast
- **Tree models** — solid green blobs without bark/leaf texture detail
- **Mipmapping** — from 20m+ distance, terrain becomes average green color
- **Low camera rate** (9-13Hz) — more motion blur between frames

ORB features need **corners and edges** with distinctive texture. The simulation provides mostly:
- Smooth gradients (sky, terrain)
- Repetitive patterns (tree canopy)
- Low contrast (green on green)

### What Helped
- Adding 40 rocks along routes (Falling Rock model has textured surface)
- Adding barrels/cones (geometric edges, high contrast orange)
- Terrain texture with dirt patches, cracks, gravel
- Result: 196 initial map points (vs 32-42 before improvements)

### What Didn't Help Enough
- Increasing nFeatures to 5000
- Lowering FAST threshold to 3
- PBR material (broke rendering — black terrain)
- 4K texture (mipmapped to same result from distance)

## Conclusion

ORB-SLAM3 is **not suitable for this Gazebo simulation** without significant visual improvements. The sparse feature-based approach fails with:
- Monotonic procedural textures
- Low-polygon tree models
- Limited camera frame rate

### Recommendations
1. **RTAB-Map** — uses dense depth matching, tolerant of low-texture environments
2. **Real-world deployment** — ORB-SLAM3 works well on real cameras with natural textures
3. **If ORB-SLAM3 required** for simulation:
   - Add high-contrast markers/signs every 10m
   - Use photorealistic textures (not procedural)
   - Increase camera to 30Hz+ with higher resolution
   - Add more geometric objects (walls, fences, signs)

## Files
- `CameraTrajectory.txt` — ORB-SLAM3 output (3,035 poses, TUM format)
- `KeyFrameTrajectory.txt` — Keyframe poses
- `gt_trajectory.csv` — Ground truth (113,429 poses)
- `trajectory_comparison.png` — Visual comparison GT vs ORB-SLAM3
- `gazebo_d435i.yaml` — ORB-SLAM3 configuration
- `associations.txt` — RGB-Depth associations (TUM format)
- `imu.txt` — IMU data (timestamp gx gy gz ax ay az)
