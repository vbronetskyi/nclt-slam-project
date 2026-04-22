# exp 75 - Gap Navigator driven by teach waypoints

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 75_gap_navigator_teach_wp*


Reactive depth-based navigator adapted from exp 31/35, driven by a teach
trajectory as its goal sequence - no Nav2, no global planner, no teach
occupancy map.

## What's in

| Component | Source | Role |
|---|---|---|
| Isaac Sim + obstacles | simulator | scene + props |
| ORB-SLAM3 RGB-D-I VIO | 3rd party | camera pose |
| `tf_wall_clock_relay_v55` | ours | publishes map->base_link from VIO+encoder |
| `visual_landmark_matcher` | ours | anchor correction against teach landmarks |
| `turnaround_supervisor` | ours | FIRE obstacle removal at 10 m from final |
| `GapNavigator` (`gap_navigator.py`) | from exp 31/35 | reactive depth->cmd_vel |
| `gap_nav_teach_client.py` | new (here) | feeds teach WPs + computes heading-error for GapNav |

## What's removed vs our custom

- No Nav2 (`planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `map_server`).
- No `teach_map.pgm` (global costmap).
- No `send_goals_hybrid` WP detour ring.
- No `pure_pursuit_path_follower`.

## The loop

```
for each WP in subsampled teach trajectory (spacing 4 m):
    while d(robot, WP) > 3 m:
        depth_image  = last /camera/depth/image_rect_raw
        desired_yaw  = atan2(WP_y - r_y, WP_x - r_x)
        heading_err  = normalize(desired_yaw - robot_yaw)
        lin, ang, _  = GapNavigator.compute_cmd_vel(depth, heading_err)
        publish /cmd_vel(lin, ang)
    REACHED; next WP
```

Per-WP timeout: 180 s (double for last 5 WPs).  Stall detection:
if robot moves < 0.3 m in 30 s outside of the final tail, skip this WP.

## Expected vs ours

| Metric | Our custom T&R | Exp 75 Gap Nav |
|---|---|---|
| Reached / total | 36 / 36 | TBD |
| Duration | 689 s | TBD |
| Final WP reached | x | TBD |
| Has global plan | x (Nav2) | ✗ |
| Has teach map | x | ✗ |
| Has teach landmarks | x | x (via matcher) |

## how to run

```bash
ROS_DOMAIN_ID=88 bash scripts/run_gap_nav.sh
# blocks until RESULT or 40-min timeout; outputs in results/run_09/
```

## Artefacts

Under `results/run_09/`: `goals.log` (client log + RESULT line),
`tf_slam.log`, `landmark_matcher.log`, `anchor_matches.csv`,
`isaac.log`, `vio.log`, `supervisor.log`, `traj_gt.csv`, `run_info.txt`.
