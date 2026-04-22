# 02_north_forest - teach

*[thesis root](../../../../../README.md) > [simulation](../../../../README.md) > isaac > routes > 02_north_forest > teach*


Source experiment: `71_teach_north_forest_accel_noise`.

Drives Husky open-loop through the deep northern forest, recording:
- Isaac bag -> `/root/isaac_tr_datasets/02_north_forest/teach/isaac_slam_1776780358/`
- Landmarks -> `teach/north_forest/landmarks.pkl` (via `visual_landmark_recorder.py`)
- Depth costmap -> `teach/north_forest/teach_map.{pgm,yaml}` (via `teach_run_depth_mapper.py`)
- Dense VIO vs GT -> `teach/north_forest/vio_pose_dense.csv` (via `vio_drift_monitor.py`)

Key run params (from `scripts/run_teach_single.sh`):
- Route: `north_forest` (registered in `/tmp/slam_routes.json`)
- IMU: synthetic with accel-noise profile
- Duration: 4500 s sim, route timeout 10000 s
- 197 WP through forest, y-peak ≈ +33 m, small 360° loop at turnaround
- Result: drift_max 0.91 m at 395 m path (see `vio_vs_gt.png` in dataset `teach_outputs/`)
