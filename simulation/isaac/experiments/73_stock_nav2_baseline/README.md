# Exp 73 - Stock Nav2 NavigateThroughPoses baseline

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 73_stock_nav2_baseline*


Drop-in replacement for our custom `send_goals_hybrid.py + pure_pursuit_path_follower.py + WP lookahead skip + 4-7 m detour ring` with **stock Nav2 behavior** - `NavigateThroughPoses` action + default `navigate_through_poses_w_replanning_and_recovery.xml` behavior tree + stock RPP controller + stock recoveries (spin, back-up, wait, drive-on-heading).

## Everything held fixed vs routes/\<NN\>/repeat

| Component | route run | exp 73 |
|---|---|---|
| Isaac Sim + obstacles | same | same |
| ORB-SLAM3 RGB-D-I VIO | same | same |
| `tf_wall_clock_relay_v55 --slam-encoder` | same | same |
| `visual_landmark_matcher` (anchor correction) | same | same |
| Nav2 `map_server` + `planner_server` (NavFn + global_costmap [static+obstacle+inflation]) | same | same |
| Nav2 `controller_server` | - (we use pure_pursuit_path_follower) | **added (RPP controller)** |
| Nav2 `behavior_server` (recoveries) | - | **added (spin, backup, drive_on_heading, wait)** |
| Nav2 `bt_navigator` (behavior tree) | - | **added (stock BT XML)** |
| `turnaround_supervisor` | same | same |
| WP feeder | `send_goals_hybrid.py` | `nav_through_poses_client.py` (sends full WP list to `/navigate_through_poses`) |

## what this measures

- Our custom `send_goals_hybrid` does:
  - per-WP cost-lookahead SKIP
  - 4-7 m detour-ring sampling around high-cost WPs
  - retry with 2× timeout + no-SKIP on last 5 WPs
- Stock Nav2 BT does:
  - global replanning on failure
  - clear-local-costmap -> spin -> back-up -> drive-on-heading -> wait recoveries
  - path-is-invalid / goal-is-reached decorators

If stock Nav2 handles the 6-prop route 09 comparably or better, our custom
detour-ring adds no value. If stock Nav2 times out / stalls / crashes into
props, the detour ring is load-bearing.

## Route

09_se_ne (SE -> NE, ~145 m, 5 props at +76.7/-15.0, +76.7/-13.9, +76.4/+9.5, +73.7/+24.5, +73.7/+25.7 - cardboxes, dumpster, barrels). Short = quick experiment turnaround.

Our non-stock run on 09: **36/36 reached, 0 skipped, 689 s** (see
`/root/isaac_tr_datasets/09_se_ne/repeat/results/repeat_run/goals.log`).

## files

```
73_stock_nav2_baseline/
├── README.md                              this file
├── config/
│   ├── nav2_stock_params.yaml             planner + controller + costmaps + BT + behaviors
│   └── nav2_launch_stock.py               launches all 5 Nav2 servers + lifecycle
├── scripts/
│   ├── run_stock_nav2.sh                  orchestrator (Isaac + VIO + matcher + stock Nav2 + supervisor + client)
│   └── nav_through_poses_client.py        reads teach traj, subsamples 4 m, calls /navigate_through_poses
└── results/
    └── run_09/
        ├── isaac.log, vio.log, tf_slam.log
        ├── nav2.log, supervisor.log, landmark_matcher.log
        ├── goals.log                      client log with RESULT line
        ├── anchor_matches.csv
        ├── traj_gt.csv                    copied from /tmp/isaac_trajectory.csv at run end
        └── run_info.txt
```

## How to run

```bash
ROS_DOMAIN_ID=86 bash scripts/run_stock_nav2.sh
# blocks until RESULT line or timeout (40 min default)
# outputs in results/run_09/
```

## result (run_09, 2026-04-22)

| Metric | routes/09 custom | **exp 73 stock** |
|---|---|---|
| Reached / total | 36 / 36 (100 %) | **3 / 36 (8 %)** |
| Skipped | 0 | 33 (BT Aborted) |
| Duration | 689 s | **85 s** (aborted) |
| Final WP reached | x | ✗ (aborted at WP 4) |
| Failure mode | - | RPP "collision ahead" loop -> clear-costmap recovery -> replan -> same collision -> BT Aborted |

Full log: `results/run_09/goals.log`, `results/run_09/nav2.log`.

### what went wrong (usefull for thesis)

Robot stalled at GT ≈ (75, −16), facing the first cardbox at (76.7, −15.0).
Repeated sequence:

1. RPP: `detected collision ahead` (forward-simulated path crosses costmap
   cell with cost ≥ 99, from depth-derived obstacle_layer).
2. Controller aborts after `failure_tolerance=0.3 s`.
3. BT triggers `clear_local_costmap` -> `clear_global_costmap` -> new plan.
4. Planner re-computes a path - but the teach WP itself is
   **2.3 m from the obstacle centre**, within inflation radius, so the
   newly-planned path still ends inside the costmap-inflated cell.
5. RPP sees collision again. Loop.

After ~80 s of recovery cycles the `error_code_names` chain in `bt_navigator`
aborts the whole NavigateThroughPoses goal.

### Why our custom pipeline handles this

`send_goals_hybrid.py::follow_waypoint` does, for every WP:
- `_lookahead_cost(x,y)` - peek the costmap cell at the WP
- if `cost ≥ LOOKAHEAD_SKIP_COST (60)` -> `_find_detour(x,y)` samples a
  4–7 m ring around the WP for a free cell and uses that as the goal instead
- falls back to re-try with 2× timeout on the last 5 WPs

Stock Nav2's behavior tree has no equivalent WP-level safety check - it
assumes every goal is reachable and relies on the planner to find a path.
When the goal itself is inside obstacle inflation, the planner cannot
produce a path that reaches the goal without crossing inflation, and the
controller refuses to drive that path.

**Take-away for thesis:** the detour-ring WP-level safety is
**load-bearing** - not a cosmetic fallback. Removing it and relying on
stock Nav2 recovery behaviors fails at the first obstacle cluster.

### Artefacts

All under `results/run_09/`: `goals.log` (client), `nav2.log` (Nav2 stack),
`tf_slam.log`, `isaac.log`, `vio.log`, `supervisor.log`,
`landmark_matcher.log`, `anchor_matches.csv`, `traj_gt.csv` (GT trajectory
from `/tmp/isaac_trajectory.csv`), `run_info.txt` (bag path).
