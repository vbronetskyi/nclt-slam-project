# exp 02: Nav2 with RGBD-only ORB-SLAM3 (mapping mode)

## Goal

Replace GT localization (exp 01) with real visual SLAM. ORB-SLAM3 RGBD mode runs in **mapping mode** (no atlas), builds a map from scratch during navigation. The relay reads `/tmp/slam_pose.txt` and computes a map->odom correction.

## setup

- Same as exp 01 (Nav2 + DWB + obstacle layer + waypoints)
- SLAM: ORB-SLAM3 RGBD-only, `rgbd_live` binary, mapping mode
- Camera: D435i intrinsics (fx=fy=320, 640×480, 10 Hz, ThDepth=160, DepthMapFactor=1000)
- Pose pipeline: Isaac writes `/tmp/isaac_pose.txt` (GT, for relay's GT mode), `rgbd_live` writes `/tmp/slam_pose.txt`. Relay converts SLAM camera frame -> world via initial offset
- Two configurations tested: `wz_max = 0.8 rad/s` (fast turning) and `wz_max = 0.4 rad/s` (slow turning)

## Results

| wz_max | distance | route % | obstacles | Y drift | lost frames |
|---|---|---|---|---|---|
| 0.8 rad/s | 137 m | 75 % | 2/4 | 10 m | 274/970 = 28 % |
| 0.4 rad/s | 104 m | 57 % | 2/4 | 5.8 m | 7/2685 = 0.9 % |

## What we learned

1. **Sharp turns kill SLAM tracking.** At wz_max=0.8, 28 % of frames are lost - the camera sees a completely new view in 0.5 seconds and feature matching fails. After lost frames, position jumps 60 m+. Jump filter (>5 m) rejects it but SLAM freezes on a stale pose.
2. Slowing turns recovers tracking. wz_max=0.4 gives 0.9 % lost frames but the run is slower and accumulates more lateral drift over the longer route.
3. **No lateral constraints on straight roads.** Monocular RGBD has metric depth from depth sensor, but the lateral ("strafe") direction is constrained only by features on the road sides. Forest with sparse features -> 5-10 m lateral drift.

## Final result (best run)

**106 m, 58 % route, 19/34 outbound waypoints reached.** 0.1 % lost frames. Cones at x=-50 bypassed. Stuck at x=-23 y=9 after tent bypass: SLAM thinks robot is on the road but it's actually drifted 7 m north and now blocked by trees.

Offline SLAM evaluation on full road recording (3651 frames, 365 s):
- 0/3651 lost (perfect tracking offline)
- ATE RMSE 7.88 m (mostly from turnaround lateral drift)
- Max lateral 13.5 m at turnaround
- Return error 1.5 m (loop closure)

## files

- `config/nav2_husky_params.yaml` - same as exp 01
- `config/rgbd_d435i_v2_mapping.yaml` - ORB-SLAM3 RGBD-only mapping config
- `scripts/run_husky_nav2.py` - adds `--use-slam` flag, launches `rgbd_live` as subprocess
- `scripts/tf_wall_clock_relay.py` - SLAM mode: reads `/tmp/slam_pose.txt`, converts SLAM camera frame to world via initial spawn offset
- `results/trajectory_plot.png`

## limitations exposed

This experiment establishes the **fundamental SLAM problem** that drives experiments 03-19:
- Mapping mode drift (this experiment)
- Atlas localization mode (exp 03)
- Sensor fusion (exp 04-08)
- SLAM frame navigation (exp 09-14)
- VIO with IMU (exp 15-19)

All later SLAM experiments are attempts to fix the lateral-drift problem visible here.
