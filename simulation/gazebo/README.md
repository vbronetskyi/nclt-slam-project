# UGV Simulation - Clearpath Husky A200

Autonomous navigation simulation of a Clearpath Husky A200 unmanned ground vehicle in a procedurally generated outdoor environment. Built with ROS 2 Jazzy, Gazebo Harmonic, Nav2, and RTAB-Map for visual SLAM evaluation.

![World overview](docs/images/world_overview.png)

![3D terrain visualization](docs/images/world_realistic.png)

## Key Specifications

| Parameter | Value |
|-----------|-------|
| World size | 220 x 150 m procedural terrain |
| Models | 370 total (297 trees, 40 fallen trees, 23 rocks, 5 buildings) |
| Robot | Clearpath Husky A200, 50 kg, skid-steer |
| RGBD camera | Intel RealSense D435i, 640x480, ~10--17 Hz actual |
| IMU | 250 Hz (Phidgets Spatial 1042) |
| LiDAR | 360 deg, 10 Hz (disabled to save GPU) |
| Ground truth | Gazebo `dynamic_pose/info`, 50 Hz, <1 cm accuracy |
| Spawn point | (-105, -8) |
| Route length | 216 m (forest, open field, village) |
| Middleware | ROS 2 Jazzy, Gazebo Harmonic |
| Navigation | Nav2 (NavFn + Regulated Pure Pursuit) |
| SLAM | RTAB-Map (visual), SLAM Toolbox (2D lidar) |

## Quick Start

### Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- NVIDIA GPU (required for EGL camera rendering in headless mode)

### Build

```bash
cd /workspace/simulation/gazebo
colcon build --symlink-install --packages-select ugv_description ugv_gazebo ugv_navigation
source install/setup.bash
```

### Launch (headless)

```bash
# Required for camera in headless mode
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Start Gazebo + Nav2 + SLAM
ros2 launch ugv_gazebo full_sim.launch.py headless:=true
```

### Web UI (click-to-drive)

```bash
python3 tools/web_nav.py
# Open http://localhost:8765
```

The web interface provides a 2D map with the robot position, SLAM overlay, camera feed (MJPEG), and click-to-drive navigation. Click anywhere on the map to send the robot to that point. A STOP button allows immediate halt.

## Project Structure

```
gazebo/
├── src/ugv_description/        Robot SDF model + URDF TF tree
├── src/ugv_gazebo/             Gazebo worlds, bridge config, launch files
├── src/ugv_navigation/         Nav2 + RTAB-Map configuration
├── tools/                      web_nav.py, drive_route.py, record_drive.py, etc.
├── scripts/                    World generators (v1, v2)
├── routes/                     Route definitions (JSON)
├── experiments/                All experiments with results
│   ├── 00_navigation_test/     First Nav2 integration test
│   ├── 01_autonomous_drive/    Route 1 autonomous drive + ORB-SLAM3
│   └── 02_slam_comparison/     RTAB-Map vs ORB-SLAM3 comparison
└── docs/                       Detailed documentation
    ├── environment.md
    ├── robot.md
    ├── simulation.md
    └── images/
```

## Experiments

| # | Experiment | Description | Key Result |
|---|-----------|-------------|------------|
| 00 | [Navigation test](experiments/00_navigation_test/) | First Nav2 integration test on the procedural terrain | Validated Nav2 stack with RPP controller |
| 01 | [Autonomous drive](experiments/01_autonomous_drive/) | 216 m route through forest, field, and village | Collected RGBD-Inertial dataset, ran ORB-SLAM3 |
| 02 | [SLAM comparison](experiments/02_slam_comparison/) | RTAB-Map vs ORB-SLAM3 evaluation | Quantitative ATE/RPE comparison on the same route |

## Documentation

- [Environment](docs/environment.md) - World generation, terrain, object placement, zones
- [Robot](docs/robot.md) - Husky A200 chassis, sensors, TF frames, differential drive
- [Simulation](docs/simulation.md) - Launch files, Gazebo-ROS bridge, Nav2 config, known issues

## References

- **ORB-SLAM3** - Campos et al., 2021 - [paper](https://arxiv.org/abs/2007.11898) - [code](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **RTAB-Map** - Labbé & Michaud, 2019 - [paper](https://doi.org/10.1177/0278364918770436) - [code](https://github.com/introlab/rtabmap)
- **Nav2** - Macenski et al., 2020 - [paper](https://arxiv.org/abs/2003.00368) - [code](https://github.com/ros-navigation/navigation2)
- **SLAM Toolbox** - Macenski & Jambrecic, 2021 - [code](https://github.com/SteveMacenski/slam_toolbox)
- **Gazebo Harmonic** - [website](https://gazebosim.org/)
- **ROS 2 Jazzy** - [website](https://docs.ros.org/en/jazzy/)
- **Clearpath Husky A200** - [product page](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
- **Intel RealSense D435i** - [product page](https://www.intelrealsense.com/depth-camera-d435i/)
