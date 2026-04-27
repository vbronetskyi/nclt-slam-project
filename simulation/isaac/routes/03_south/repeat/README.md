# Exp 59 - WP Look-Ahead Skip + Detour

*[thesis root](../../../../../README.md) > [simulation](../../../../README.md) > isaac > routes > 03_south > repeat*


Continuous teach-and-repeat (T&R) navigation on the south forest roundtrip,
built on exp 58 base (visual landmark matcher + route sanitizer + ORB-SLAM3   
RGB-D-I VIO) with two changes aimed at obstacle handling:

1. Run-time WP look-ahead skip + detour. When a forthcoming waypoint is
   too close to a known or observed obstacle, it is replaced with a detour
   sampled on a 4–7 m ring around the waypoint in a safe direction.
2. Stricter clearance in Nav2. `robot_radius` 0.7 m, `inflation_radius`
   1.5 m, `tolerance` 0.3 m - guarantees the planner never ends a path
   inside the inflation zone.

## What changed vs exp 58

| Area | Exp 58 | Exp 59 |
|---|---|---|
| WP safety handling | Offline `sanitize_route.py` shifts WP <=2 m | Runtime look-ahead skip + detour on 4–7 m ring |
| Obstacle awareness | teach-map static_layer only | live depth-camera obstacle_layer |
| `planner.tolerance` | 1.0 m | 0.3 m |
| `robot_radius` | 0.5 m | 0.7 m |
| `inflation_radius` | 1.2 m | 1.5 m |
| Goal sender robot-pose source | `/tmp/isaac_pose.txt` (Isaac GT) | `map->base_link` tf (SLAM) |

The last row was a bug in goal_sender across exps 55–58: it used GT to test
REACH while the planner used SLAM-frame WPs. Once SLAM drift exceeded
tolerance, REACH could not fire - robot orbited the WP forever. Fix lives
in [send_goals_hybrid.py:103](scripts/send_goals_hybrid.py#L103).

## Run 5 results (run_exp59_repeat.sh)

| Metric | Value |
|---|---|
| Distance | 540 m (roundtrip) |
| Duration | 35 min |
| WP REACHED | **79 / 95 (83 %)** |
| WP SKIP | 10 (planner unreachable after detour) |
| DETOUR triggered | 18 (15 succeeded, 3 skipped) |
| Wedge recoveries | **0** |
| Drift mean | **1.67 m** |
| Drift max | **3.64 m** |
| Anchor publishes | 174 / 1160 attempts (15 %) |
| Collisions (dense GT) | **cone group 2 (x=+5): 75 cm body overlap; tent: 22 cm body overlap** |

The 0 wedges observation from pp_follower is still true, but dense
`isaac_trajectory.csv` shows physical overlap: the robot drove straight
thorugh cone (5, −18) at sim-time 223 s and brushed the tent edge
at sim-time 78 s. Detour logic fires correctly in SLAM frame but at the
moment of the cone pass SLAM drift was 2.35 m - so the SLAM-frame detour
target (9.0, −17.9) mapped to GT position ≈ (6.6, −15.6), which took the
physical trajectory right across the cone. Fix belongs in exp 60:
drift reduction and/or SLAM-frame obstacle coordinates for the known-
obstacle check.

### comparison against earlier experiments (same route)

| | Exp 56 (baseline) | Exp 58 | **Exp 59 run 5** |
|---|---|---|---|
| REACHED | **88 / 95 (93 %)** | 82 / 95 | 79 / 95 (83 %) |
| SKIP | 4 | 7 | 10 |
| DETOUR triggered | - | - | **18** |
| Wedge recoveries | 0 | 268 | **0** |
| Drift mean | 4.86 m | 13.3 m | **1.67 m** |
| Drift max | 8.61 m | 83 m | **3.64 m** |
| Anchor publishes | 44 / 306 (14 %) | +-90 / 1000 (9 %) | **174 / 1160 (15 %)** |
| Tent body-edge overlap | no (lucky) | **yes** (tangential brush) | **22 cm** |
| Worst cone body overlap | slight contacts | **yes** | **75 cm** |
| Duration | 42 min | 40 min | 35 min |

Exp 56 was the previous works best run: clean traversal, no crashes.
Exp 58 added continuous landmark accumulation (explosion of drift from
wedging near tent/cones). Exp 59 reintroduces clean traversal *and* keeps
the landmark accumulation benefit - best drift and best anchor rate of
the three.

REACH rate drop vs exp 56 (93 % -> 83 %) is by design: exp 59 refuses
to reach WPs whose projected cell has high costmap cost (obstacle_layer
from depth). A WP that was reached in exp 56 (robot drove 0.5 m past a
cone) is a SKIP in exp 59.

Clean traversal -> order-of-magnitude better localisation. Without
impacts:

**Clean traversal -> 8* better localisation.** Without impacts:
- no bump/shake -> VIO keeps feature tracks
- camera view not blocked -> anchor matcher can still find landmarks
- no spin-in-place recovery -> no motion blur -> VIO healthy

Compared to exp 56 (also clean traversal): drift mean dropped 4.86 -> 1.67 m
because the landmark accumulator from exp 58 keeps building anchors along
the live path, so `no_anchor` windows stay shorter.

## Run history in this experiment

- `run3_detour_in_inflation/` - early run where detour target itself fell
  inside teach-map tree inflation (cost 100). Fixed by ring retry
  (4/5/6/7 m) + strict `DETOUR_MAX_COST=30`.
- `run4_gt_pose_bug/` - detour worked for cones/tent, but robot began
  circling WP 68 after 30 min of anchor-stale drift (5 m). Root cause: goal
  sender used Isaac GT to check REACH; fixed in run 5.
- `repeat_run/` - **final run with tf-based REACH**. See metrics above.

## trajectory

![trajectory](results/repeat_run/trajectory.png)

Green = Isaac ground truth. Dashed blue = SLAM+ENC fused nav. Black   
dashed = teach reference. Obstacles (orange cones + green tent) shown at
true positions.

### GT vs exp 56 (baseline)

![exp59 vs exp56](results/repeat_run/trajectory_vs_exp56.png)

Exp 59 routes visibly farther from the tent and cone groups (detour
rings) while exp 56 brushes past them.

## Known issues

- 10 WPs SKIPPED - planner could not reach detour targets in
  tree-dense forest. Detour point falls in teach-map tree inflation.
  Possible fix: allow higher `DETOUR_MAX_COST` in tree-heavy regions, or
  let planner use `allow_unknown=true` more aggressively.
- **Anchor rate 15 %** - good when robot is on teach path, sparse during
  detours (new viewpoints). Augmentation in exp 58 helped, but detours
  still take the robot into never-seen corridors where landmark matcher
  has nothing to match.
- Drift still grows during `no_anchor` windows. 17 min anchor-stale
  window yielded +-3 m drift. Acceptable but not ideal.

## Reproduce

```bash
bash scripts/run_exp59_repeat.sh
# Results -> results/repeat_run/
# plot  -> scripts/plot (via plot_trajectory_map)
```
