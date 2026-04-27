# 03_south - south roundtrip

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 03_south*


## Route layout

- Spawn: (-94.9, -6.0)
- Turnaround: (69.7, -5.1)
- **Teach path length (one-way-ish)**: +-320 m
- **Obstacles at repeat**: 9 cones (3 groups) + 1 tent on outbound

## teach run

VIO drift (teach): _n/a - older pipeline with GT-only TF logging_

Teach artefacts in `/root/isaac_tr_datasets/03_south/teach/teach_outputs/`:
`landmarks.pkl` (ORB visual landmarks), `teach_map.yaml/pgm`
(depth-derived occupancy), `vio_pose_dense.csv` (dense VIO+GT samples,
regenerated from bag groundtruth for legacy routes).

VIO-vs-GT teach plot: _not available for 01/02/03 legacy routes (older teach pipeline)_.

## Repeat run (our custom T&R)

Pipeline: Isaac Sim + obstacles -> ORB-SLAM3 RGB-D-Inertial VIO ->
`tf_wall_clock_relay_v55 --slam-encoder` + `visual_landmark_matcher` ->
Nav2 planner + depth obstacle_layer + `send_goals_hybrid`
(WP-lookahead + 4–7 m detour-ring) + `pure_pursuit_path_follower` +
`turnaround_supervisor` (drops obstacles at 10 m from the turnaround).

Detailed run report: [`repeat/README.md`](repeat/README.md).

Per-stack artefacts under `results/repeat_run/` (symlink to
`/root/isaac_tr_datasets/03_south/repeat/results/repeat_run/`):
`goals.log`, `traj_gt.csv`, `nav2.log`, `pp_follower.log`, `tf_slam.log`,
`supervisor.log`, `anchor_matches.csv`, `repeat_result.png`,
`baseline_compare.png`.
## baseline comparison

Three stacks, same teach WP list (4 m spacing), same obstacles, same simulator.
- **reach** = min GT distance to turnaround (x <= 10 m)
- **return** = GT end-pose distance to spawn (x <= 10 m, coverage >= 50 %)
- coverage = teach WPs within 3 m of any GT sample
- **drift** = `|published_pose − GT|` mean / p95 / max (m)

| stack | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| **our custom T&R** | 5.7 m x | 5.9 m x | 85/96 (89%) | 2.00 / 3.43 / 3.57 |
| exp 74 stock Nav2 | 149.9 m ✗ | 21.3 m ✗ | 12/96 (12%) | 1.71 / 2.47 / 4.18 |
| exp 76 RGB-D only (no IMU) | 94.0 m ✗ | 72.5 m ✗ | 25/96 (26%) | 3.32 / 5.32 / 6.04 |

Overview plot (all 3 GT trajectories on the scene map): [`baseline_compare.png`](results/repeat_run/baseline_compare.png)

Per-stack artefacts (goals.log, tf_slam.log, traj_gt.csv, ...):
- our custom: `results/repeat_run/`
- exp 74 stock Nav2: `../../experiments/74_pure_stock_nav2_baseline/results/run_03_south/`
- exp 76 RGB-D no IMU: `../../experiments/76_rgbd_no_imu_ours/results/run_03_south/`

