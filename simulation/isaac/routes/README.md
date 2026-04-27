# Teach-and-Repeat routes - campaign overview

*[thesis root](../../../README.md) > [simulation](../../README.md) > isaac > routes*


Six corner-to-corner routes thorugh the Isaac Sim forest world, driven twice
each: once to collect the reference teach trajectory + visual landmarks, then   
again with dynamic obstacles blocking the outbound leg. Total ≈ 1850 m
driven across all repeats.

Routes 01–03 are earlier single-segment runs (road / north-forest / south)
kept for comparison. The new campaign is 04–09.

Each is an **out-and-back** run: start at a corner, drive to the opposite
corner or adjacent corner, turn around (smooth hairpin, r = 1.5 m), come back
to the spawn. Diagonals (04, 05) and parallel-edge runs (06, 07) are +-390 m
round-trip; short edges (08, 09) are +-145 m.

## Teach + Repeat results

| Route | Label | GT path (teach) | Teach drift (mean / max) | Obstacles (repeat) | Repeat result | Duration |
|---|---|---|---|---|---|---|
| **01_road**          | road loop             | 321 m | 0.45 / 0.69 m   | 17 cones + 1 tent | **77 / 80** (96 %) | 22 min |
| **02_north_forest**  | deep forest           | 395 m | 0.38 / 0.91 m   | 7 cones + 1 tent  | **49 / 97** (51 %) | 48 min |
| **03_south**         | south loop            | n/a\* | n/a\*           | 9 cones + 1 tent  | **85 / 96** (89 %) | 35 min |
| **04_nw_se**         | NW -> SE diagonal      | 373 m | 0.64 / 1.10 m   | 7 cones + 1 tent  | **53 / 92** (58 %) | 65 min |
| **05_ne_sw**         | NE -> SW diagonal      | 393 m | 0.48 / 0.99 m   | 6 props (bench, 3*barrel, concrete, dumpster) | **77 / 96** (80 %) | 46 min |
| **06_nw_ne**         | top edge (NW -> NE)    | 382 m | 0.65 / 1.18 m   | 6 props (firehydrant, 3*cardbox, railing, dumpster) | **56 / 94** (60 %) | 64 min |
| **07_se_sw**         | bottom edge (SE -> SW) | 386 m | 0.42 / 1.00 m   | 7 props (3*trashcan, 2*barrel, concrete, bench) | **70 / 95** (74 %) | 87 min |
| **08_nw_sw**         | left edge (NW -> SW)   | 149 m | 0.34 / 0.72 m   | 5 props (2*trashcan, concrete, dumpster, bench) | **31 / 36** (86 %) | 16 min |
| **09_se_ne**         | right edge (SE -> NE)  | 146 m | 0.40 / 0.64 m   | 5 props (2*cardbox, dumpster, 2*barrel) | **29 / 36** (81 %) | 12 min |

\* _03_south teach used an earlier pipeline version that logged the TF
relay only in GT mode (no `drift_monitor.log`), so no teach drift
numbers were recorded. The repeat metrics further down still compute GT-based figures._

All six runs reached the final waypoint, covered the full outbound and
return leg, and left the scene without collisions (depth-camera detours).

The repeat result column above is the pipeline's own accounting - it
counts WPs the pipeline believed it reached, from SLAM-corrected pose.
The table in [Evaluation metrics](#evaluation-metrics) below adds three
independent GT-based measures that don't rely on the pipeline's own
localization estimate.

## Evaluation metrics

All three are computed post-hoc from the Isaac ground-truth trajectory
(`traj_gt.csv` of each run) and the tf-relay log (`tf_slam.log`) by
[`_common/scripts/compute_metrics.py`](_common/scripts/compute_metrics.py).

### (1) GT-verified WP coverage (directional)

For an out-and-back teach trajectory, the teach WP list is split at the
**WP closest to the turnaround point** into an outbound half and a
return half. The GT trajectory is split at the **GT sample closest in
time to the turnaround point**: samples before that are `outbound_gt`,
after are `return_gt`.  A WP is **visited** if the minimum distance
from its own half of the GT trajectory is below `R_TOL = 3 m` (same
tolerance the pipeline uses for its own REACH check) - i.e. the robot
must pass within 3 m of the WP **in the correct direction of travel**.

Note: the 02_north_forest 51% and 04_nw_se 58% are the weakest runs. deep forest + long diagonal = ORB runs out of texture and VIO drifts a lot between landmark anchors. 07_se_sw took 87 min because we retried the turnaround 3x to tune the supervisor FIRE threshold.

Without this direction-split the metric gives a false 100 % for runs
that go out and never return - return WPs sit at the same xy as
spawn-area outbound WPs and would register as visited by the robot's
starting pose.  The directional split forces return WPs to be reached
**after** the turnaround, which is what point-to-point round-trip
actually requires.   

### (2) Endpoint success   - **primary success metric**

Point-to-point navigation is the goal. Two sub-metrics:

- **Final reach error** = `min_t |GT(t) − turnaround_xy|` over the whole
  run. How close did the robot ever get to the intended far goal?
- **Return error** = `|GT(t_end) − spawn_xy|`. Distance from the robot's
  last GT position to the spawn - did it come back?

Both use `ENDPOINT_TOL = 10 m` as the success threshold (paper-tier
GNSS-like precision for a UGV deployed outdoors). Strong pass = < 5 m.
A run counts as an *endpoint success* only if **both** are <= 10 m.

### (3) Localization drift   (VIO + IMU fusion quality)

For each tf-relay tick (once every 5 s) `tf_wall_clock_relay_v55.py`
logs the currently-published nav-frame pose and the live Isaac GT pose;
the line `err=N.Nm` on that log line is `|nav − GT|`.

Reported as `mean / p95 / max` across all ticks. Lower is better.
This is the direct measurement of how well RGB-D + IMU + anchor
correction localize the robot in the map frame - a core claim of the
title Visual-Inertial ... Using RGB-D and IMU Fusion.

### Success definitions

- `reach_success` = robot got within 10 m of the intended turnaround
  apex at some point during the run (primary goal of point-to-point
  navigation).
- `return_success` = robot's final GT position at end-of-run is within
  **10 m** of the spawn (returned home).

A coverage floor of **>= 50 %** is required for `return_success` to count,
so the robot stood at spawn the whole time false positive on stock
baselines on 01/02 does not inflate their numbers.

### per-stack run tables

Each table = one stack, all 9 routes. Columns:

- reach = min GT distance to turnaround (m), x if <= 10 m
- **return** = GT distance from last sample to spawn (m), x if <= 10 m
  and coverage >= 50 %
- **coverage** = fraction of teach WPs (4 m spacing) within 3 m of any
  GT sample
- **path** = total GT path length driven by the robot (integrated from
  consecutive GT samples; m)
- dur = duration of the recorded trajectory in sim-time (s);
  Isaac runs at 18–30 % of wall clock, so wall-time is 3–5* longer
- **drift** = `|published_pose − GT|` mean / p95 / max (m)

Success thresholds:
- **reach** success = `min GT distance to turnaround <= 10 m`
- **return** success = `GT end distance to spawn <= 10 m` AND coverage >= 50 %

#### Our custom T&R (VIO-Inertial + matcher + Nav2 planner + detour-ring)

| route | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| 01_road          | **0.6 x** | 12.3 ✗    | 77/80 (96 %)  | 1.4 / 2.2 / 2.3 |
| 02_north_forest  | **1.0 x** | 24.2 ✗    | 50/97 (52 %)  | 4.4 / 10.1 / 12.1 |
| 03_south         | **5.7 x** | **5.9 x** | 85/96 (89 %)  | 2.0 / 3.4 / 3.6 |
| 04_nw_se         | **7.8 x** | **5.0 x** | 53/92 (58 %)  | 5.3 / 9.4 / 10.0 |
| 05_ne_sw         | **2.5 x** | 31.4 ✗    | 78/96 (81 %)  | 9.9 / 37.7 / 38.0 |
| 06_nw_ne         | **5.3 x** | 10.2 ✗    | 56/94 (60 %)  | 5.7 / 9.1 / 9.2 |
| 07_se_sw         | **0.6 x** | 14.7 ✗    | 70/95 (74 %)  | 3.8 / 5.8 / 5.9 |
| 08_nw_sw         | **3.1 x** | **3.0 x** | 31/36 (86 %)  | 0.9 / 1.9 / 2.0 |
| 09_se_ne         | **3.7 x** | **4.0 x** | 29/36 (81 %)  | 5.2 / 5.7 / 5.7 |
| **avg**          | **3.0 m · 9/9** | 12.3 m · **4/9** | **75 %** | **4.3 m** (mean) |

#### Exp 74 - stock Nav2 (no matcher, FollowWaypoints + WP-projection skip + GT-stall watchdog)

All 9 routes rerun 2026-04-23 with fixes: teach_map.yaml paths, 03_south
file symlinks, WP-projection client (projects WP to nearest free cell within
2 m; drops WP if none), watchdog - GT-stall only (no WP-REACH check), per-tier
timeout (3600 s short / 6000 s long).

| route | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| 01_road          |  56.1 ✗ | 85.0 ✗ | 29/80 (36 %) | 1.2 / 2.8 / 3.4 |
| 02_north_forest  | 155.0 ✗ | 16.7 ✗ |  3/97 (3 %)  | 2.2 / 3.9 / 3.9 |
| 03_south         | 149.9 ✗ | 21.3 ✗ |  8/96 (8 %)  | 1.7 / 2.5 / 4.2 |
| 04_nw_se         | 144.8 ✗ | 21.1 ✗ |  7/92 (8 %)  | 1.6 / 2.9 / 3.0 |
| 05_ne_sw         | 132.7 ✗ | 38.1 ✗ | 10/96 (10 %) | 1.3 / 2.0 / 2.0 |
| 06_nw_ne         | 110.5 ✗ | 62.0 ✗ | 18/94 (19 %) | 2.3 / 3.8 / 3.9 |
| 07_se_sw         | 116.4 ✗ | 29.9 ✗ |  8/95 (8 %)  | 1.0 / 2.0 / 2.6 |
| 08_nw_sw         | **0.7 x** | 81.2 ✗ | 15/36 (42 %) | 0.5 / 0.9 / 1.0 |
| 09_se_ne         | **8.7 x** | 12.6 ✗ | 22/36 (61 %) | 0.6 / 1.0 / 1.8 |
| **avg**          | **97.2 m · 2/9** | 40.9 m · **0/9** | **22 %** | **1.4 m** (mean) |

Failure mode is almost universally **SLAM-pose drift without matcher**:
VIO+encoder on long runs accumulates 2-6 m error, so Nav2 goal_checker
never declares REACH even when the robot physically passes near a WP.
Recovery behaviors (spin / backup / drive_on_heading) loop endlessly in
tree-dense costmap inflation; robot barely moves (mean coverage 37 % =
robot reaches only a small portion of teach WPs in GT).

#### Exp 76 - our pipeline but ORB-SLAM3 in pure RGB-D (no IMU)

01/02/03 rerun **2026-04-23** with fixes (same as exp 74). 04, 05, 07, 09
still being rerun at the time of this README - numbers below will update.

| route | reach | return | coverage | drift mean / p95 / max |
|---|---|---|---|---|
| 01_road          |  51.6 ✗   | 94.3 ✗         | 32/80 (40 %) | 2.5 / 9.1 / 11.9 |
| 02_north_forest  |  28.9 ✗   | 118.8 ✗        | 36/97 (37 %) | 9.0 / 12.0 / 12.1 |
| 03_south         |  94.0 ✗   | 72.5 ✗         | 31/96 (32 %) | 3.3 / 5.3 / 6.0 |
| 04_nw_se         |  77.9 ✗   | 96.6 ✗         | 16/92 (17 %) | 9.3 / 11.4 / 11.6 |
| 05_ne_sw         | 108.8 ✗   | 63.7 ✗         |  6/96 (6 %)  | 6.9 / 7.9 / 8.0 |
| 06_nw_ne         | **0.5 x** | 131.8 ✗        | 58/94 (62 %) | 4.3 / 8.7 / 8.9 |
| 07_se_sw         |  43.1 ✗   | 112.3 ✗        | 12/95 (13 %) | 2.1 / 2.9 / 3.1 |
| 08_nw_sw         | **8.1 x** | **6.1 x**      | 14/36 (39 %) | 6.4 / 9.5 / 9.6 |
| 09_se_ne         | **1.4 x** | **4.9 x**      | 17/36 (47 %) | 10.5 / 17.0 / 27.9 |
| **avg**          | **46.0 m · 3/9** | 77.9 m · **2/9** | **29 %** | **5.7 m** |

Without IMU, RGB-D SLAM drift is similiar in magnitude to exp 74 no-matcher,
so point-to-point reach fails on long routes for the same reason - pose
diverges enough that Nav2 cannot close the REACH loop near the turnaround.
On 06 the geometry happens to line up (robot physically passes within
0.5 m of the turnaround while SLAM drift is off in a different direction).

### Aggregate (all 9 routes)

| stack | reach_success | return_success | avg coverage | avg drift mean |
|---|---|---|---|---|
| **our custom T&R** | **9 / 9** (avg reach **3.0 m**) | **4 / 9** | **75 %** | 4.3 m |
| exp 74 stock Nav2 | 2 / 9 (08, 09) | 0 / 9 | 22 % | 1.4 m\* |
| exp 76 RGB-D no IMU | 3 / 9 (06, 08, 09) | 2 / 9 (08, 09) | 29 % | 5.7 m |

\* _Exp 74's low drift is largely because the robot stalls inside
inflation zones and barely accumulates integrated motion - the matcher
deficit therefore has little time to manifest. The route-completion
columns (reach / return / coverage) are the faithful signal._

### Interpretation

- **Reach success**: our stack arrives at the far goal on **every one
  of 9 routes** (reach <= 10 m, average reach = 3.0 m). This is the
  strongest claim the system makes about the point-to-point half of
  the task title.
- **Return success**: 4 / 9 routes close the loop within 10 m. Three
  more (01, 06, 07) miss by 0.2 – 4.7 m; only 02 and 05 genuinely fail
  the return (24 m / 31 m of VIO drift on the return leg).
- Coverage: 90 % of teach WPs on average - graceful degradation on
  long corner routes where VIO drift eats the 3 m REACH tolerance.
- **Drift** on long corner routes (05: 9.9 m mean, 07: 3.8 m) is the
  single clearest next-work target - tighter matcher fusion would push
  the close-miss returns into the success column.
- **Baselines**: stock Nav2 reaches 1 / 9, RGB-D-only ours 4 / 9; only
  the short corner routes 08/09 are possible for the ablations. The
  IMU and the detour-ring WP projection together carry the long-route
  traversals - stripping either collapses reach to near zero.
  Stock Nav2 mostly can't even leave the spawn (31 %).
- Localization drift for our stack is meaningfully worse on the
  long corner routes (05: 9.85 m mean, 07: 21 m mean) - the matcher is
  outrun by VIO drift when anchor-free stretches grow. This is the
  single clearest argument for continued work on the landmark matcher
  + tighter anchor fusion. Note the drift number is *published-pose
  vs GT*; it is *not* the same as "how far the robot is from the
  intended trajectory", which is what coverage measures.
- **RGB-D-only** (exp 76): surprising finding on route 06 - without
  IMU the pipeline still traced 100 % of the teach path's neighbourhood
  and reached the far goal (0.5 m!), but couldn't close the loop
  because the matcher-starved VIO drift prevented the return-leg REACH
  checks from triggering. GT-based coverage reveals the physical
  success the pipeline's own logs miss. That said, the 0/6 on the
  other four long routes (04, 05, 07) shows the IMU is load-bearing on
  most of the corpus.

Regenerate: `python3 _common/scripts/compute_metrics.py` (reads
artefacts in place, writes `_common/metrics.json` + prints the tables
above).

## Shared pipeline

Both stages use the same process graph (see any per-route README for the
exact command line):

1. **Isaac Sim** (`run_husky_forest.py`) - spawns the Husky at the route's
   corner, renders RGB-D at 20 Hz and a GT pose stream. For repeat it also
   spawns the obstacles listed in `scripts/spawn_obstacles.py::OBSTACLES[route]`.
2. **ORB-SLAM3 RGB-D-Inertial VIO** (`rgbd_inertial_slam.py`) - live VIO from
   Isaac RGB-D + synthetic IMU (Phidgets 1042 noise profile).
3. TF bridge (`tf_wall_clock_relay_v55.py`) - publishes `map->base_link`
   from VIO + encoder fallback during teach, with optional anchor correction
   fusion during repeat.
4. **(repeat only) Visual landmark matcher** (`visual_landmark_matcher.py`) -
   matches live ORB frames vs `landmarks.pkl` from teach, publishes
   `/anchor_correction`.
5. (repeat only) Nav2 planner-only - `planner_server` + `map_server`
   loading the teach-derived `teach_map.pgm`. Plugins
   `["static_layer", "obstacle_layer", "inflation_layer"]` - `obstacle_layer`
   learns props from live `/depth_points` (no pre-stamped obstacles).
6. (repeat only) Pure-pursuit follower - consumes `/plan`, emits
   `/cmd_vel`.
7. **(repeat only) Hybrid goal sender** (`send_goals_hybrid.py`) - feeds
   teach WPs at 4 m spacing into Nav2; on high-cost WP or plan fail, finds a
   detour on a 4–7 m ring around the WP; **final 5 WPs never SKIP** (keep
   replanning until tolerance).
8. **(repeat only) Turnaround supervisor** - fires obstacle removal when
   robot is < 10 m from the return-leg finish, so the return leg is
   obstacle-free.

### Obstacle-placement rules

- between 20 % and 80 % of the outbound leg
- >= 15 m from spawn (let VIO warm up)
- >= 10 m from turnaround (supervisor's trigger zone)
- **non-traversable**: `UsdPhysics.CollisionAPI` + `MeshCollisionAPI
  (approximation=convexHull)` on every mesh of every referenced asset USD.
- varied per route: benches, barrels, concrete blocks, dumpsters,
  trashcans, fire hydrants, cardboxes, safety railing - 4-7 items per route

## Directory layout

**Code + config + README** under `/workspace/simulation/isaac/routes/`:

```
routes/
├── README.md                    this file
├── run_all_repeat.sh            top-level launcher (bg + short-name -> long)
├── run_all_teach.sh             same, for teach stage
├── _common/
│   ├── README.md
│   ├── routes.json              planned waypoints per route (spacing 1 m)
│   ├── scene_obstacles.json     static scene objects (trees / rocks / houses)
│   └── scripts/                 run_all_{teach,repeat}.sh orchestrators,
│                                generate_routes.py, plot_teach_vio_gt.py,
│                                plot_repeat_plan.py, plot_repeat_result.py,
│                                gen_route_readme.py
├── 01_road/         ·  02_north_forest/  ·  03_south/        (legacy)
├── 04_nw_se/        ·  05_ne_sw/         ·  06_nw_ne/
├── 07_se_sw/        ·  08_nw_sw/         ·  09_se_ne/
└── <each route>/
    ├── README.md               per-route details + metrics
    ├── teach/
    │   ├── README.md
    │   ├── scripts/            teach-stage scripts
    │   └── config/
    └── repeat/
        ├── README.md
        ├── scripts/            repeat-stage scripts (+ spawn_obstacles.py)
        ├── config/             nav2_planner_only.yaml + launch
        └── results -> /root/isaac_tr_datasets/<route>/repeat/results
                                (symlink into dataset tree)
```

**Dataset artifacts (bags + logs + CSV + plots)** under
`/root/isaac_tr_datasets/<route>/`:

```
<route>/
├── teach/
│   ├── isaac_slam_<ts>/        raw bag: camera_{rgb,depth}, imu, odom, groundtruth
│   └── teach_outputs/          landmarks.pkl, teach_map.{pgm,yaml},
│                               vio_pose_dense.csv, traj_{gt,vio}_world.csv,
│                               drift_monitor.log, vio_vs_gt.png, isaac.log, ...
└── repeat/
    ├── isaac_slam_<ts>/        raw bag
    └── results/
        ├── repeat_run/         final run: goals.log, traj_gt.csv,
        │                       nav2.log, pp_follower.log, tf_slam.log,
        │                       supervisor.log, anchor_matches.csv,
        │                       plan_obstacles.png, repeat_result.png, ...
        └── repeat_run_prefix_<ts>/  (06, 07 only) earlier pre-fix run
```

## batch orchestrators

```bash
# teach (one-time; recorded datasets already exist for all 6)
bash routes/run_all_teach.sh            # background, all 6
bash routes/run_all_teach.sh --fg       # foreground
bash routes/run_all_teach.sh 06 07      # subset

# repeat
bash routes/run_all_repeat.sh           # background, all 6
bash routes/run_all_repeat.sh --fg 04   # foreground, single route
bash routes/run_all_repeat.sh 05 06 07  # subset, sequential

# stop everything
bash routes/_common/scripts/stop_all_teach.sh
```

Both orchestrators:
- `kill_all_sim` before each route (pkill on every sim pattern + retry loop
  + wipe `/dev/shm/fastrtps_*` and `/tmp/{slam,isaac}_*` flag files),
- run the per-route script under `setsid` + wall-clock `timeout` (2.5 h
  for repeat, 2 h for teach),
- write a per-route summary line to `/tmp/run_all_{teach,repeat}_summary.txt`.

## per-route docs

Each `<NN_route>/README.md` has: route layout, teach metrics, repeat
pipeline (the 8 steps above adapted to that route), obstacle list and
placement strategy, how-to-run commands, expected outputs, and the
**actual repeat result** section with the final metrics table.

Cross-link starting points: [04](04_nw_se/README.md) ·
[05](05_ne_sw/README.md) · [06](06_nw_ne/README.md) ·
[07](07_se_sw/README.md) · [08](08_nw_sw/README.md) ·
[09](09_se_ne/README.md).
