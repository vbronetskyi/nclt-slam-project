# Exp 74 - Pure stock Nav2 baseline (no matcher, supervisor kept)

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 74_pure_stock_nav2_baseline*


Compared to exp 73 (navigation-logic ablation), this one **removes the
visual_landmark_matcher** - the key anchor-correction contribution of our
T&R pipeline. VIO is left to drift uncorrected.

point of this exp is to show what a "stock" config does if you drop all our
T&R bits but keep the simulator, SLAM and Nav2 basics. thesis baseline.

## what's in vs out

| Component | Where it comes from | exp 74 |
|---|---|---|
| Isaac Sim + obstacles | simulator | ON |
| ORB-SLAM3 RGB-D-I VIO | third-party SLAM | ON |
| `tf_wall_clock_relay_v55 --slam-encoder` | ours (pose publisher) | ON |
| `visual_landmark_matcher` | **ours** (teach-landmark anchor correction) | **OFF** |
| `turnaround_supervisor` | ours (obstacle-drop signal) | ON (fair) |
| `send_goals_hybrid` (WP detour ring) | ours | OFF |
| `pure_pursuit_path_follower` | ours | OFF |
| Nav2 `planner_server` + `map_server` | stock | ON |
| Nav2 `controller_server` (RPP) | stock | ON, with v3 fix: `rotate_to_heading=true`, `allow_reversing=true` |
| Nav2 `bt_navigator` (stock BT XML) | stock | ON |
| Nav2 `behavior_server` (spin/backup/wait/drive_on_heading) | stock | ON |
| Nav2 `waypoint_follower` (`stop_on_failure: false`) | stock | ON |
| `waypoint_follower_client.py` | trivial 80-line ActionClient | ON |

### What "without matcher" means in practice

- `tf_wall_clock_relay_v55` runs, but no one publishes `/anchor_correction`.
- It stays in `no_anchor` regime for the entire run.
- Published `map->base_link` = pure VIO + encoder fusion, with whatever drift
  accumulates from VIO.
- Teach drift on 09_se_ne was mean 0.40 m / max 0.64 m over 145 m - small,
  so VIO alone may be usable here, but we'll measure.

## turnaround fix (applied to both v3 and this exp)

Exp 73 v2 overshoot at NE corner by ~14 m was caused by RPP controller being
forward-only in its config (`use_rotate_to_heading: false`, `allow_reversing: false`).
The canonical Nav2 fix is to let RPP rotate in place when the planned path
diverges sharply from robot heading.

Config deltas:
```yaml
FollowPath:
  use_rotate_to_heading: true
  rotate_to_heading_min_angle: 0.785   # 45 deg
  max_angular_accel: 2.0
  allow_reversing: true
```

## how to run

```bash
ROS_DOMAIN_ID=87 bash scripts/run_pure_stock_nav2.sh
# blocks until RESULT line or timeout (40 min)
# outputs in results/run_09/
```

## Result (v4, 2026-04-22)

| Metric | Our custom T&R | **Exp 74 pure stock** |
|---|---|---|
| Reached / total | 36 / 36 (100 %) | **22 / 35 (63 %)** |
| Skipped | 0 | 13 |
| Duration | 689 s | **1048 s** (+52 %) |
| Final WP reached | x | ✗ (last WPs SKIP'd) |
| Matcher / anchor correction | ON | **OFF** |
| Turnaround behaviour | supervisor FIRE at 10 m | supervisor FIRE at 10 m, but RPP overshoots |

v4 settings (after v1-v3 tuning):
- `use_rotate_to_heading: false`, `allow_reversing: false` - stock forward-only RPP
- `planner.tolerance: 1.0` (was 3.0 in v1: caused single-pose trivial plans -> BT wait loop)
- `progress_checker: 0.3 m / 30 s` (loosened from 0.5 m / 10 s)
- `failure_tolerance: 3.0 s` (was 0.3)
- Client skips initial WPs already within 3.5 m of robot pose

Comparison plots:
- [`compare_routes.png`](results/compare_routes.png) - GT trajectories + obstacles
- [`compare_localisation.png`](results/compare_localisation.png) - anchor-corrected vs pure VIO+encoder

### What this result shows

1. Turnaround overshoot: same as exp 73 v2 - at NE corner, RPP cannot U-turn with
   `use_rotate_to_heading=false`; robot overshoots by ~10 m north of apex before
   coming back.  Supervisor fires obstacle removal at overshoot point, return
   leg is clean.
2. **Return leg works** (unlike exp 73 v2 where all return WPs failed):
   loose progress checker + lower planner tolerance let robot keep trying
   after intermittent failures on return.
- Localisation is OK for this short (145 m) route: tf_slam regime stays
   `no_anchor` whole run; VIO drift stays <1 m on GT vs nav-frame comparison.
   On longer routes (04–07, 370+ m) this would fall apart.

## expected outputs

All under `results/run_09/`:
- `goals.log` - waypoint_follower_client log with RESULT line and missed WP list
- `nav2.log` - full Nav2 stack logs
- `tf_slam.log` - tf_relay in `no_anchor` regime (drift visible)
- `isaac.log`, `vio.log`, `supervisor.log`
- `traj_gt.csv` - GT trajectory
- `run_info.txt` - bag path
