# 01_road - road loop

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > routes > 01_road*


## route layout

- **Spawn**: (-80.0, -1.4)
- **Turnaround**: (70.5, -2.7)
- Teach path length (one-way-ish): +-321 m
- **Obstacles at repeat**: 17 traffic cones (3 groups) + 1 tent on outbound

## teach run

VIO drift (teach): **0.45 / 0.69 m**

Teach artefacts in `/root/isaac_tr_datasets/01_road/teach/teach_outputs/`:
`landmarks.pkl` (ORB visual landmarks), `teach_map.yaml/pgm`
(depth-derived occupancy), `vio_pose_dense.csv` (dense VIO+GT samples,
regenerated from bag groundtruth for legacy routes).

VIO-vs-GT teach plot: _not available for 01/02/03 legacy routes (older teach pipeline)_.

## repeat run (our custom T&R)

Pipeline: Isaac Sim + obstacles -> ORB-SLAM3 RGB-D-Inertial VIO ->
`tf_wall_clock_relay_v55 --slam-encoder` + `visual_landmark_matcher` ->
Nav2 planner + depth obstacle_layer + `send_goals_hybrid`
(WP-lookahead + 4–7 m detour-ring) + `pure_pursuit_path_follower` +
`turnaround_supervisor` (drops obstacles at 10 m from the turnaround).

Detailed run report: [`repeat/README.md`](repeat/README.md).

Per-stack artefacts under `results/repeat_run/` (symlink to
`/root/isaac_tr_datasets/01_road/repeat/results/repeat_run/`):
`goals.log`, `traj_gt.csv`, `nav2.log`, `pp_follower.log`, `tf_slam.log`,
`supervisor.log`, `anchor_matches.csv`, `repeat_result.png`,
`baseline_compare.png`.
## Baseline comparison

Three stacks, same teach WP list (4 m spacing), same obstacles, same simulator.
- **reach** = min GT distance to turnaround (x <= 10 m)
- **return** = GT end-pose distance to spawn (x <= 10 m, coverage >= 50 %)
- **coverage** = teach WPs within 3 m of any GT sample
- **drift** = `|published_pose − GT|` mean / p95 / max (m)

| stack | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| **our custom T&R** | 0.6 m x | 12.3 m ✗ | 77/80 (96%) | 1.36 / 2.16 / 2.27 |
| exp 74 stock Nav2 | 56.1 m ✗ | 85.0 m ✗ | 26/80 (32%) | 1.24 / 2.80 / 3.40 |
| exp 76 RGB-D only (no IMU) | 51.6 m ✗ | 94.3 m ✗ | 24/80 (30%) | 2.47 / 9.14 / 11.85 |

Overview plot (all 3 GT trajectories on the scene map): [`baseline_compare.png`](results/repeat_run/baseline_compare.png)

Per-stack artefacts (goals.log, tf_slam.log, traj_gt.csv, ...):
- our custom: `results/repeat_run/`
- exp 74 stock Nav2: `../../experiments/74_pure_stock_nav2_baseline/results/run_01_road/`
- exp 76 RGB-D no IMU: `../../experiments/76_rgbd_no_imu_ours/results/run_01_road/`

