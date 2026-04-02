# Exp 56 - visual teach-and-repeat v2 (projection cap + anchor stack)

## summary

Exp 56 inherits the visual teach-and-repeat stack from exp 55 and adds
one targeted fix: a **1 m cap on proactive WP projection**.  One other
change (ground-feature-only matching) was tried and reverted after
run 1 showed it made things worse.

**Headline result (run 2):**
- 1 390 m driven on the 94-WP roundtrip (vs 477 m in exp 55)
- **83 / 94 WPs reached (88 %)** - essentially a full loop
- Final GT (−84, −23) - back at spawn area
- Max drift 8.6 m over 1.4 km (2.2 × better than exp 55)
- Only 1 WP skipped, 20 wedge-recoveries

![trajectory](results/repeat_run/trajectory_map.png)
![err + regime](results/repeat_run/err_and_regime.png)

## motivation (from exp 55 analysis)

Exp 55 succeeded on the outbound half (477 m, past turnaround at x=60)
but stalled on the return because:

- Matcher silent for long periods (up to 27 min) in cone and deep-forest
  zones -> drift grew unchecked to 15-18 m.
- Once SLAM lost tracking, internal relocalisation jumped pose by 50 m.
- The proactive WP projector was aggressively shoving waypoints
  several metres from the teach path into free cells; the robot then
  followed a route visibly different from teach -> camera view drifted
  from teach -> matcher failed even more.

The feedback loop `projection -> off-path drive -> camera view change
-> no anchor matches -> drift -> SLAM loss` was the dominant failure mode.

## Changes vs exp 55

### B. Projection shift cap (1 m)  x kept

In `send_goals_hybrid.py::_project_wp()`:

```python
shift = math.hypot(nx - x, ny - y)
if shift > self.PROJ_MAX_SHIFT_M:   # 1.0 m
    return x, y, True   # leave WP unchanged
```

Rationale: keep the robot on the teach path.  If the BFS would move the
WP more than 1 m, prefer leaving it as-is - the planner will either
find a tight path or fail (and goal sender skips via the
skip-on-5-fails logic added in exp 55).

This single change flipped the feedback loop: now the robot follows a
path close to teach, camera views match teach, matcher still operates
(same ~14 % rate) and the **accumulated drift stays bounded**.

### A. Ground-feature-only matching  ✗ tried & reverted

Idea: keep only ORB keypoints in the bottom half of the image
(v > 180) to reject sky/distant-tree features that change between
teach and repeat.

What went wrong in run 1:

- 39 / 128 teach landmarks had <10 ground features -> dropped entirely.
- Remaining landmarks had mean 37 features (down from 75).
- Fewer features -> lower descriptor specificity -> more false-positive
  matches passing the shift≤5 m consistency check.
- Anchor rate dropped 13 % -> 5 %; SLAM eventually lost tracking.

Conclusion: ground-only restriction discards too much information in a
forest scene.  Full-frame matching with the 5 m consistency check
remains the right default.

## inherited from exp 55 (unchanged)

All scene fixes, navigation logic and localisation stack are identical
to exp 55:

- Scene: 357 shrubs re-placed on terrain (0.15 m colliders), 154
  grass/leaves cover items, 6 roadside tree cylinders corrected.
- `visual_landmark_recorder.py`: full-frame ORB, depth 0.5-15 m,
  terrain-aware camera pose, 2 m min-disp between landmarks.
- `visual_landmark_matcher.py`: BFMatcher crossCheck, PnP-RANSAC, heading
  filter ±90°, consistency shift ≤5 m.
- `tf_wall_clock_relay_v55.py`: anchor regimes (strong/ok/no_anchor)
  with v54 adaptive SLAM/encoder fallback when anchor stale >10 s.
- `pure_pursuit_path_follower.py`: 0.8 m/s cruise / 0.4 near obstacle /
  0.15 lethal, plan-based proximity, wedge-recovery (2.5 s back-up if
  robot displaces <0.15 m in 4 s).
- `send_goals_hybrid.py`: proactive WP projection (costmap-driven BFS),
  skip-on-5-fails for infeasible WPs.

## Results - run 2 (1 390 m, 83 WPs)

| Metric                   | Exp 55 run 3 | Exp 56 run 2 |
|--------------------------|-------------:|-------------:|
| Distance driven          | 477 m        | **1 390 m**  |
| WPs reached              | 19 / 94      | **83 / 94**  |
| WPs skipped (plan fail)  | 20           | **1**        |
| WPs timeout              | 4            | 3            |
| Wedge recoveries         | 23           | 20           |
| Anchor match rate        | 13.1 %       | 14.4 %       |
| SLAM err mean / max      | 4.98 / 18.69 | **4.87 / 8.61** |
| Encoder err max          | 9.29 m       | 6.09 m       |
| Final drift              | 9.09 m       | 8.23 m       |
| Final GT position        | (72, 0.3)    | **(−84, −23)** (≈ spawn) |

### route coverage

GT x reached from `−88` (spawn) to `+66` (past turnaround at x=60) and
back to `−84`.  In the route parameterisation this covers WPs 0 through
~83 of 94.  The last 11 WPs are the tail of the return path - robot
stopped short of full close-of-loop because we killed the run (not
because the stack failed).

### Anchor behaviour

- 305 match attempts -> 44 published (14.4 %)
- 1867 / 1879 control ticks (99.4 %) in `no_anchor` regime
- Only 6 ticks in `ok`, 0 in `strong`
- The robot completed the route almost entirely under the v54 adaptive
  SLAM+encoder fallback, with anchor contributing in short bursts

The adaptive fallback is clearly the workhorse in this run.  Matcher
is still a bottleneck - 14 % fire rate is not enough to correct drift
continuously.  But because drift stayed bounded (≤8.6 m) and encoder
err stayed ≤6 m, the fallback alone delivered a usable nav pose for
the full 1.4 km.

## remaining problems

1. **Matcher rate still low** (14 %).  80 % of attempts fail
   `no_pnp_accept` - not enough geometric inliers in forest scenes
   between teach and repeat despite the same scene.  Root cause most
   likely: ORB feature positions shift slightly with viewpoint, and the
   crossCheck matcher is strict.
2. **No recovery from SLAM loss.**  In run 2 the robot stopped moving
   a few times when SLAM temporarily lost track (lost=4, slam_f=13696
   at end).  Adaptive fallback took over, but the underlying SLAM didn't
   self-recover.
3. **Last 11 WPs.**  We killed the run manually at `(−84, −23)` - the
   trajectory was still consistent, anchor age 9 500 s.  A longer
   `goal_timeout` might have finished the loop.

## Directory layout

```
56_projection_cap_1m/
├── scripts/
│   ├── visual_landmark_recorder.py        (teach-time ORB + 3D)
│   ├── visual_landmark_matcher.py         (repeat-time PnP + anchor)
│   ├── tf_wall_clock_relay_v55.py         (fused TF with anchor+v54 fallback)
│   ├── pure_pursuit_path_follower.py      (0.8 m/s cruise, wedge-recovery)
│   ├── send_goals_hybrid.py               (proactive projection + skip-on-fail)
│   ├── analyze_repeat.py                  (trajectory_map + err_and_regime plots)
│   ├── run_exp56_teach.sh                 (teach run)
│   └── run_exp56_repeat.sh                (repeat run)
├── config/
│   ├── nav2_planner_only.yaml             (NavFn, tolerance=1.0)
│   ├── nav2_launch_hybrid.py
│   └── vio_th160.yaml                     (ORB-SLAM3 config, loopClosing=0)
├── teach/
│   ├── south_teach_map.pgm / .yaml        (depth-based occupancy)
│   └── south_landmarks.pkl                (128 visual landmarks)
└── results/
    ├── run1_ground_filter_bad/             (ground filter attempt, reverted)
    └── repeat_run/                         (run 2 - 1 390 m, 83 WPs)
```

## Reproducing

```bash
# teach is already included (copied from exp 55); to redo:
bash scripts/run_exp56_teach.sh

# repeat with all current fixes:
bash scripts/run_exp56_repeat.sh

# analysis:
python3 scripts/analyze_repeat.py
```

## next experiment (-> exp 57?)

Two directions could continue to improve match rate and close the last
11 WPs:

1. Denser teach map - redo teach with `--min-disp 0.5` instead of
   2 m to get ~250 landmarks on a ~450 m route (closer to robot pose
   at any moment -> less viewpoint disparity in matching).
2. **Depth-scan ICP** as a second anchor channel - match current depth
   point cloud against teach occupancy grid.  Independent of ORB, so
   unaffected by cones changing the scene.
