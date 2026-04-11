# 02_north_forest - deep forest roundtrip

## Route layout

- **Spawn**: (-84.4, 4.5)
- **Turnaround**: (70.4, -2.3)
- **Teach path length (one-way-ish)**: ~395 m
- **Obstacles at repeat**: 7 cones (3 groups) + 1 tent on outbound

## Teach run

VIO drift (teach): **0.38 / 0.91 m**

Teach artefacts in `/root/isaac_tr_datasets/02_north_forest/teach/teach_outputs/`:
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
`/root/isaac_tr_datasets/02_north_forest/repeat/results/repeat_run/`):
`goals.log`, `traj_gt.csv`, `nav2.log`, `pp_follower.log`, `tf_slam.log`,
`supervisor.log`, `anchor_matches.csv`, `repeat_result.png`,
`baseline_compare.png`.
## Baseline comparison

Three stacks, same teach WP list (4 m spacing), same obstacles, same simulator.
- **reach** = min GT distance to turnaround (x ≤ 10 m)
- **return** = GT end-pose distance to spawn (x ≤ 10 m, coverage ≥ 50 %)
- **coverage** = teach WPs within 3 m of any GT sample
- **drift** = `|published_pose − GT|` mean / p95 / max (m)

| stack | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| **our custom T&R** | 1.0 m x | 24.2 m ✗ | 49/97 (51%) | 4.39 / 10.11 / 12.14 |
| exp 74 stock Nav2 | 155.0 m ✗ | 16.7 m ✗ | 3/97 (3%) | 2.24 / 3.89 / 3.94 |
| exp 76 RGB-D only (no IMU) | 28.9 m ✗ | 118.8 m ✗ | 24/97 (25%) | 8.96 / 11.95 / 12.10 |

Overview plot (all 3 GT trajectories on the scene map): [`baseline_compare.png`](results/repeat_run/baseline_compare.png)

Per-stack artefacts (goals.log, tf_slam.log, traj_gt.csv, ...):
- our custom: `results/repeat_run/`
- exp 74 stock Nav2: `../../experiments/74_pure_stock_nav2_baseline/results/run_02_north_forest/`
- exp 76 RGB-D no IMU: `../../experiments/76_rgbd_no_imu_ours/results/run_02_north_forest/`

