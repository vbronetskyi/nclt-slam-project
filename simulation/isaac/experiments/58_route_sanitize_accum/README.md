# exp 58 - visual T&R v4: WP sanitization + continuous landmark accumulation

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 58_route_sanitize_accum*


## Motivation (from exp 56 retrospection)

Exp 56 reached 1 390 m and 83 / 94 WPs - the best result so far - but
still had two structural weaknesses:

1. **Tent collision** (and once brush-against-a-shrub) because the
   pre-recorded route's waypoint 16 at (−44.7, −38.5) is literally
   inside the tent footprint.  Nav2's planner tolerance of 1 m means
   the plan can end near the cone/tent *inflation* and robot drives
   in while wedge-recovery bails out.
2. **Matcher silent for long periods** (anchor rate 14 %), so the
   fused nav pose runs on adaptive SLAM/encoder fallback for most of
   the roundtrip, drift slowly grows to 8 m.

Exp 58 addresses both with two targeted changes, both staying within
the T&R paradigm (no hybrid teach, no ground-feature filter, no
depth-ICP).

## Changes vs exp 56

### 1. Offline WP sanitization (`sanitize_route.py`)

Before the repeat run starts, read the original trajectory, the teach
occupancy map, and the known cone/tent positions.  For each waypoint:

- Compute distance to nearest occupied cell (teach map + cones + tent
  rectangle).
- If distance ≥ 1.7 m (robot_radius 0.5 + inflation 1.2), keep WP.
- Else BFS outward up to 3 m for a cell with ≥ 1.7 m clearance.  If
  found, replace the WP.  If not found, mark SKIP.

Output `teach/sanitized_route.csv` that `send_goals_hybrid.py`
consumes instead of the raw trajectory.

Deterministic effect on the south route:
- 81 / 95 WPs unchanged (safe already)
- 14 WPs shifted (max shift 2.10 m)
- 0 WPs skipped (everything found a safe cell within 3 m)

Crucially WP 16 at the tent center shifted **2.0 m south below the
tent**.  Same for WP 80 on the return.  Cone-wall WPs shifted by
1.0-1.8 m to the side of the walls.

The planner can no longer legitimately end a path inside any
obstacle's inflation.  Tent/cone collisions become impossible at the
route level.

### 2. Continuous landmark accumulation (in `visual_landmark_matcher.py`)

When the matcher fails to find a good anchor (no_candidates,
no_pnp_accept, or consistency_fail), it now checks:

- Has no anchor been published in the last 5 s?  (i.e. we're drifting
  in `no_anchor` regime)
- Is the nearest existing teach landmark > 5 m away from current VIO
  position?  (i.e. coverage gap)
- Does the current frame have ≥ 30 valid 3D keypoints?

If all three, the matcher **records the current frame as a new
landmark** with its VIO-derived camera pose.  The landmark list grows
organically.  On SIGTERM (or normal shutdown), the augmented list is
pickled to `teach/south_landmarks_augmented.pkl`.

Rationale: matcher failing repeatedly over long stretches means that
region genuinely lacks coverage.  Next time the robot passes (same run
or a later one), matches will succeed there because we added a
landmark the first time through.  Classic "lifelong T&R" pattern.

Concern about self-consistency: accumulated landmarks use the current
(possibly drifted) VIO pose, so they're self-consistant in the **VIO
frame**, not GT.  In the same run, a future frame matching against the
accumulated landmark will *not* give drift-free truth - but it will
stabilise the VIO pose against *itself*, reducing drift variance.  Over
multiple runs, accumulated landmarks naturally align to the initial
teach frame (because VIO roughly re-converges to GT during early
strong-anchor periods).

### 3. Inherited from exp 56 (unchanged)

- Scene fixes (shrub z-correction, cover population, roadside trees)
- `send_goals_hybrid.py` with proactive WP projection + skip-on-fail
- `pure_pursuit_path_follower.py` with 0.8 m/s cruise + wedge recovery
- `tf_wall_clock_relay_v55.py` with adaptive SLAM/encoder fallback
- ORB full-frame matching + crossCheck + heading filter ± 90°

## Expected outcome

- **Tent / cone collisions: zero** (sanitization guarantees)
- Anchor rate: baseline 14 % + accumulation over time.  First run
  should see modest improvement; successive runs more.
- **Accumulated pkl growth**: 128 -> 130-150 landmarks in first run
  (only coverage gaps get filled), stabilising at ~200 after 2-3 runs.

## Files added/changed

```
58_route_sanitize_accum/
├── scripts/
│   ├── sanitize_route.py                 <- NEW
│   ├── visual_landmark_matcher.py        <- +accumulation, SIGTERM save
│   └── run_exp58_repeat.sh               <- calls sanitize then uses output
├── teach/                                (copied from exp 56)
│   └── sanitized_route.csv               (generated at run start)
```

## Reproducing

```bash
bash scripts/run_exp58_repeat.sh          # sanitizes + starts repeat
# teach re-use is unchanged; south_landmarks.pkl from exp 56 is the seed
```

## results - run 1 (sanitized route + continuous accumulation)

| Metric                   | Exp 56 run 2 | **Exp 58 run 1** |
|--------------------------|-------------:|-----------------:|
| Distance driven          | 1 390 m      | **1 363 m** |
| WPs reached              | 83 / 94      | **56 / 94** |
| WPs timeout              | 4            | **29** |
| WPs skipped              | 1            | 1 |
| Wedge recoveries         | 20           | **268**   note️ |
| Anchor match rate        | 14.4 %       | 7.8 % |
| Accumulated landmarks    | - (not supported) | **20 new** |
| SLAM err mean / max      | 4.87 / 8.61  | 13.3 / 83.0  note️ |
| Encoder err max          | 6.09 m       | 21.6 m |
| Final drift              | 8.23 m       | 15.1 m |
| **Final GT position**    | (−84, −23)   | **(−88, −22)** (≈ spawn) |

![trajectory](results/repeat_run/trajectory_map.png)
![err + regime](results/repeat_run/err_and_regime.png)

### What worked

- **Tent safe-shift verified**: WP 16 sanitization moved the waypoint
  2.0 m south of the tent centre.  Robot followed planned path without
  hitting the tent footprint.  No tent-side wedge events.
- Landmark accumulation active: 20 new landmarks added to the
  live pkl during the run, saved to
  `teach/south_landmarks_augmented.pkl` (721 KB vs 529 KB original).
- **Full roundtrip completed** in terms of GT coverage:
  max GT x = 80 (past turnaround at 60), final GT x = −88 (back at
  spawn).  Robot physically made the full loop.

### What went worse than exp 56

- **Wedge count 268 vs 20** - 13× more.  Most wedges occured at
  the cone groups.  Sanitization shifts cone WPs to positions
  **just at the edge of inflation** (1.5-1.8 m from cone centre).
  Robot drives to WP, still ends up brushing against the cone because
  `send_goals_hybrid.py` accepts it as REACHED at 3 m tolerance -
  and the approach line passes tangent to the cone.
- **Catastrophic SLAM spike (err max 83 m)**: a single event around the
  middle of the return; SLAM lost tracking (`lost=74`), relocated to a
  wrong keyframe, re-aligned later.  Encoder err peaked at 21.6 m during
  the same interval because the 268 wedges' reverse-direction
  integrations accumulated fake distance.
- **29 WP timeouts** (vs 4 in exp 56).  When a plan to a sanitized
  cone-group WP repeatedly fails, the 300 s timeout fires.  Many
  timeouts around cones 1 & 2.

### Analysis (matches user observation)

User observed on run: *"робот їхав біля тенту впритик"*.
Shifting a WP 2 m from the obstacle gives ~1 m visual clearance but
the approach line still passes tangentially close - **looks like the
robot is brushing the obstacle**, which is visually
indistinguishable from a collision even if physically OK.

Sanitization as implemented in exp 58 **prevents WP-inside-obstacle**
(the catastrophic tent-collision pattern from exp 52-56) but does
**not** guarantee good clearance along the approach trajectory.  The
cone groups exhibit the same issue: WPs shifted just enough to exit
inflation, robot scrapes past.

## Next direction (-> exp 59)

Stored in memory (nav_wp_lookahead_skip):

1. **Look-ahead WP feasibility check**: in `send_goals_hybrid`, when
   accepting WP i, pre-check WPs i+1..i+N against the live costmap.
   If any sits in `cost ≥ inscribed`, SKIP it early - don't wait for
   planner failure.
2. **Route-level detour**: when a WP is skipped, insert a detour
   node ≥ 2.5 m from the obstacle centroid so the robot **visibly
   routes around** instead of squeezing past.
3. **Conservative WP skip treshold**: instead of "shift to nearest
   1.7 m-safe cell", **skip** any WP whose distance to obstacle is
   < 2.5 m.

This would trade some route fidelity (more skipped WPs on narrow
corridors) for a visibly confident drive path.
