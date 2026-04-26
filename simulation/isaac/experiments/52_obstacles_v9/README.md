# Exp 52 v9 - honest teach-and-repeat with depth-sensed obstacles

Run the v9 hybrid pipeline (Nav2 planner_server + custom pure-pursuit
follower + ORB-SLAM3 VIO + synthetic IMU) as a real-world teach-and-repeat:

- **Teach**: robot drives the south roundtrip route **without** obstacles   
  The only map it builds is a 2-D occupancy grid from the depth camera
  (via Bresenham ray-casting on /depth_points in the `map` frame).
- **Repeat**: robot drives the same route **with** cones placed in three
  groups. Nav2's `global_costmap.obstacle_layer` reads live...

## результат

| Path                                        | Purpose |
|---------------------------------------------|---------|
| `scripts/run_exp52_teach.sh`                | Teach phase launcher (no obstacles, no Nav2; depth mapper running) |
| `scripts/run_exp52_repeat.sh`               | Repeat phase launcher (cones + Nav2 + supervisor + recorders) |
| `scripts/teach_run_depth_mapper.py`         | Builds south_teach_map.pgm/yaml from /depth_points + TF (log-odds, Bresenham) |
| `scripts/costmap_snapshotter.py`            | Saves full `/global_costmap/costmap` as .npy every 5 s + `snapshots_summary.csv` |
| `scripts/plan_logger.py`                    | Saves every `/plan` as CSV + `plans_summary.csv` |

## проблеми

- cone group 1

The cones in group 1 sit at `(−18, −23), (−18, −24), (−18, −25), (−18, −26)`.
The pre-recorded route passes **thorugh** this wall - WP 25 is at
`(−18.1, −24.4)`, right between cones 2 and 3.
