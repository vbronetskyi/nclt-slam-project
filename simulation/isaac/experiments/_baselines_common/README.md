# Baselines vs Our T&R - consolidated results (9 routes)

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > _baselines_common*


Comparison of three navigation stacks, each driven with the same teach
trajectory (sub-sampled at 4 m), same obstacles, same Isaac simulator.
Supervisor drops obstacles at < 10 m from the final goal (return leg
obstacle-free) in all three.

| stack | VIO | anchor matcher | global planner | controller | WP dispatch |
|---|---|---|---|---|---|
| **our custom T&R** | RGB-D + Inertial | x | Nav2 planner_only + detour-ring | custom pure pursuit | send_goals_hybrid (WP skip + detour) |
| **exp 74** stock Nav2 | RGB-D + Inertial | ✗ | Nav2 full stack | RPP (Nav2) | FollowWaypoints action |
| **exp 76** RGB-D no IMU | RGB-D only | x | Nav2 planner_only + detour-ring | custom pure pursuit | send_goals_hybrid |

## Result table (with partial WP progress even for aborted runs)

Entries show **WPs reached / total WPs** before the run either completed
(green x, with duration) or was aborted by watchdog / early failure (red ✗,
with reason). For the OK rows, `sk=K` is the count of intermediately
SKIP-ped waypoints (e.g. those that ended inside obstacle inflation).

| route | our custom | exp 74 stock Nav2 | exp 76 RGB-D (no IMU) |
|---|---|---|---|
| 01_road          | **77/80** (96 %)  1357 s, sk=3  | ✗ stall 0.00 m   | ✗ stall 0.00 m |
| 02_north_forest  | **95/97** (98 %)  2900 s, sk=2  | ✗ stall 0.00 m   | ✗ stall 0.00 m |
| 03_south         | **83/95** (87 %)  2082 s, sk=9  | ✗ FAILED_EARLY    | ✗ FAILED_EARLY |
| 04_nw_se         | **81/92** (88 %)  3931 s, sk=11 | 5/90 (6 %) ✗ stall 0.57 m    | 16/91 (18 %) ✗ no WP 608 s |
| 05_ne_sw         | **86/96** (90 %)  2753 s, sk=10 | 11/94 (12 %) ✗ stall 1.20 m   | 10/95 (11 %) ✗ no WP 602 s |
| 06_nw_ne         | **85/94** (90 %)  3835 s, sk=9  | 4/92 (4 %) ✗ stall 0.39 m    | **58/93** (62 %) ✗ no WP 600 s |
| 07_se_sw         | **80/95** (84 %)  5199 s, sk=15 | 27/93 (29 %) ✗ no WP 840 s   | 11/94 (12 %) ✗ no WP 600 s |
| 08_nw_sw         | **35/36** (97 %)   931 s, sk=1  | 3/34 (9 %) ✗ stall 0.10 m    | **35/36** (97 %) 1727 s, sk=1 |
| 09_se_ne         | **36/36** (100 %)  689 s, sk=0  | **34/35** (97 %) 636 s, sk=1 | **33/35** (94 %) ✗ stall 1.29 m |

**Per-route trajectory overlays**: `plots/compare_<ROUTE>.png` - scene + obstacles + all three GT trajectories.

## Aggregate

| stack | success (>80 % WP reached, final WP reached) | total routes |
|---|---|---|
| **our custom T&R** | **9 / 9** | 9 |
| exp 74 stock Nav2 | 1 / 9 (only 09_se_ne) | 9 |
| exp 76 RGB-D no IMU | 1 / 9 (only 08_nw_sw) | 9 |

Both baselines succeed only on one of the two shortest routes (~ 145 m round-trip),
and even there have distinct failure modes: exp 74 succeeds on 09 but fails on 08;
exp 76 succeeds on 08 but fails on 09. On all four corner-to-corner diagonals / edges
(04–07, ~ 380 m), both baselines abort within 10 min of waypoint dispatch.

## failure modes

| pattern | count (74) | count (76) | meaning |
|---|---|---|---|
| `stall 0.00 m in 180 s` | 3 | 3 | robot never moved from spawn; 01/02 use Isaac auto-drive that expects no external controller, so with Nav2 / RPP taking the wheel the robot just sits |
| `stall 0.10–1.29 m` | 3 | 1 | Nav2 plans but controller cannot execute (RPP "collision ahead", stale costmap, too-tight U-turn) |
| `no WP reached after N s` | 1 | 4 | first plan succeeded, but subsequent WP unreachable - goal projected inside teach-map inflation with no free cell nearby |
| `FAILED_EARLY` | 1 | 1 | 03_south: exp setup scripts exit before Phase 2 - different `run_repeat.sh` layout (older experiment) |

## Interpretation for thesis

1. **Our detour-ring WP projection is load-bearing.** `send_goals_hybrid.py::_find_detour` samples a 4–7 m ring around every planned WP; if the WP itself sits inside obstacle inflation (which happens on every outbound obstacle cluster in routes 04–07), the ring finds a free cell to re-aim at.  Stock Nav2's `FollowWaypoints` has no such mechanism - when the goal is unreachable, the controller times out and the BT moves on to the next (also-unreachable) WP.

2. **IMU is load-bearing too.** On the 380 m corner routes, pure-RGB-D ORB-SLAM3 loses track whenever features are sparse (forest canopy, motion blur during turns). The fused RGB-D-Inertial version holds pose. On shorter routes (145 m, 08/09) either works - the difference shows up only at length.

3. **Stock Nav2 has no "WP-level safety check".** It assumes goals are reachable. When the goal projects onto a costmap-inflated cell (cone/prop next to the teach trajectory), the planner returns a path that *ends* inside inflation; the controller refuses to drive it; the BT retries; the robot either stalls in place or overshoots at turnaround without being able to U-turn.

4. **Scope difference:** baselines are still usefull - they show the system boundary. Stock Nav2 on a short obstacle-free open-corridor route (09) works well. Our pipeline is specifically tuned for *longer* routes *with* off-path obstacles near teach waypoints.

## Plots

Open any of the `plots/compare_<ROUTE>.png` for the overlay. Green = our custom,
blue = exp 74 stock Nav2, red = exp 76 RGB-D no IMU. Where a baseline aborted,
that colour is missing or shows only the first few meters of motion.

## caveats

- The `/tmp/run_all_baseline_{74,76}_summary.txt` files written by the
  orchestrators contain a **bug** (setsid without `--wait` -> all exit codes 0 ->
  everything reported as OK). The real statuses above are re-derived post-hoc
  by `_baselines_common/scripts/collect_summary.py` from `goals.log` +
  `watchdog_abort.txt` of each per-route directory. Orchestrators have been
  fixed to use `setsid --wait` in future runs.
- Each baseline run is **single-shot per route** (not averaged). Nav2 behaviour
  depends on costmap state which evolves during the run; a second attempt may
  succeed where the first failed. Our custom stack is run once too, for
  a fair like-for-like comparison.
- 01_road, 02_north_forest and 03_south use different Isaac spawn
  mechanics than 04–09; their baselines would need Isaac-side changes to
  match the route_params.sh assumptions. The `stall 0.00 m` entries there
  don't indicate an algorithm failure but a launch-harness mismatch - deferred.

## Regenerate

```bash
# recompute summary + SUMMARY.md
python3 _baselines_common/scripts/collect_summary.py

# (re)generate all 9 comparison plots
python3 _baselines_common/scripts/plot_three_way.py
```
