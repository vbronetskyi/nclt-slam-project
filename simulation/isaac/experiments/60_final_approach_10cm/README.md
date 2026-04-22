# Exp 60 - Final Aproach + 10 cm Clearance Spec

Builds on exp 59. Two classes of problem surfaced there and are
targeted here:

1. **Robot spins and stops short of the route endpoint.** Final WP 94
   landed inside teach-map tree inflation so it was detoured 3–4 m
   away. After `WP 94 REACHED` the goal sender exited silently, leaving
   pure pursuit chasing a stale last plan - robot spun near LETHAL
   costmap cells until Isaac was shut down.
2. **Detour logic fires in SLAM frame but at 2–3 m drift the physical
   trajectory still clips the obstacle.** Dense GT from run 59/run 5
   shows 75 cm body overlap with cone (5, −18) and 22 cm with the tent.

## changes vs exp 59

| Area | Exp 59 | Exp 60 |
|---|---|---|
| Clearance spec (body edge to obstacle edge) | 20 cm | **10 cm** |
| `robot_radius` (Nav2) | 0.7 m | 0.6 m |
| `inflation_radius` | 1.5 m | 1.0 m |
| `KNOWN_CLEARANCE_M` (goal sender) | 0.9 m | 0.6 m |
| Detour ring radii | 4/5/6/7 m | 2/3/4/5 m (tighter, stay close to original WP) |
| Final-WP detour cost budget | 30 | **80** (teach-map tree phantoms allowed) |
| Stop after RESULT | - | goal sender publishes empty `Path` + zero `Twist` |

Other bits carried over from exp 59: tf-based REACH check, look-ahead
skip, hardcoded known-obstacle positions, continuous landmark
accumulator from exp 58.

## Motivation

With `KNOWN_CLEARANCE_M=0.6` a WP is flagged unsafe when it lies within
0.6 m of a cone or tent edge. 0.6 = robot half-width 0.5 + 10 cm
spec margin.

This is aggressive: a 10 cm physical margin only holds if SLAM drift is
well below 10 cm at the moment of the pass. In tree-dense stretches
(15–20 min no-anchor) drift can reach 2–3 m - so this run is a
"best case if localisation holds" experiment rather than a guarantee.

## how to run

```bash
bash scripts/run_exp60_repeat.sh
# Results -> results/repeat_run/
```

## run 1 results

| Metric | Exp 59 run 5 | **Exp 60 run 1** |
|---|---|---|
| REACHED | 79/95 (83 %) | **81/95 (85 %)** |
| SKIP | 10 | 11 |
| DETOUR | 18 | 11 |
| Wedge recoveries | 0 | **0** |
| Drift mean | 1.67 m | 3.11 m |
| Drift max | 3.64 m | 5.49 m |
| Anchor publishes | 174 / 1160 (15 %) | 47 / 337 (14 %) |
| Tent body-edge clearance | −22 cm (overlap) | **+151 cm** x |
| Worst cone body overlap | −75 cm | −58 cm |
| Clean STOP after RESULT | no (spins) | **yes** x |
| Duration | 35 min | 32 min |

### What's better
- **STOP after RESULT works.** Empty `Path` + zero `Twist` fixes the
  "spins indefinitely near last plan" behaviour. pp_follower logs
  `Empty path received - STOP` on finish and the wheels halt.
- **Tent is properly bypassed** (+151 cm body-edge clearance). The
  22 cm overlap from exp 59 is gone.
- **+2 pp REACH rate** (85 % vs 83 %) despite tighter spec.

### What didn't improve
- **Cone (5, −19) still clipped** by 58 cm (was 75 cm). Root cause:
  `KNOWN_CONES` are hardcoded in GT frame but the safety check is
  applied to WPs in SLAM/teach frame. At ~1.5 m drift during that pass,
  the SLAM-frame WP looks "safe" but physical robot clips the cone.
- **Drift is larger than exp 59** (3.11 m vs 1.67 m mean). Tighter ring
  radii (2–5 m) keep detours closer to the teach path, but the planner
  more often rejects those targets -> more SKIPs (11 vs 10) -> robot
  spent more time off-path -> fewer anchor matches.

![exp60 vs exp59](results/repeat_run/trajectory_vs_exp59.png)

## take-aways

1. 10 cm spec is drift-limited. Needs physical drift ≤ 10 cm at the
   moment of the pass. 15 min `no_anchor` drove drift to 5.5 m in this
   run - not reachable without denser anchors or online relocalisation.
2. **Safety check frame must match planner frame.** `KNOWN_CONES` are
   in GT; the check is applied to SLAM-frame WPs; the planner uses
   SLAM-frame costmap. Pick one frame and stay in it.
3. Ring 4–7 m > 2–5 m for detour success. Tighter ring put candidate
   points inside neighbouring tree inflation - the planner rejected
   them, producing more SKIPs and pushing the robot off-route.

## Ideas to improve (for exp 61+)

These are ordered by expected payoff; each is a distinct experiment.

### A. Smoother, less-zigzag trajectory

Symptom: every detour produces a 4–5 m sideways jog followed by a
re-join. The resulting GT track has visible zig-zag around each
obstacle group.

1. **Merge consecutive detours.** If WP `i` and `i+1` both trigger the
   same `near_cone` reason, plan a single bypass through one intermediate
   point on the safe side of the whole group, not two independent
   detours.
2. **Offset the whole teach segment around an obstacle**, not a single
   WP. When cone group is detected at `x=5`, shift every WP within
   ±8 m of it by a perpendicular offset computed from the group's
   centroid. Keeps curvature low.
3. **Use a curvature-aware planner** (`SmacPlanner` with Reeds-Shepp or
   Hybrid-A\*). `NavfnPlanner` is grid-shortest-path, which is why plans
   end with a sharp turn at every WP.
4. **Pre-smooth the teach CSV** before WP extraction. The input
   trajectory itself has high-frequency noise from VIO; a Savitzky-Golay
   pass would give a cleaner reference that the robot can follow more
   fluidly.

### B. Drift reduction (prerequisite for ≤ 10 cm spec)

1. **Anchor density.** The accumulator in exp 58 helps but still leaves
   15-min gaps in the forest middle. Lower the 5-m spacing threshold to
   2 m; allow accumulation when `no_anchor > 2 s` instead of `> 5 s`.
2. Active relocalisation. If drift estimate exceeds 2 m AND we are
   close to a cone/tent, slow to 0.1 m/s and hold until the matcher
   publishes. Trades throughput for safety near obstacles.
3. **Drift-aware clearance.** Scale `KNOWN_CLEARANCE_M` by current
   drift estimate: `clearance = 0.1 + 1.5 × drift_estimate`. Tight where
   we know we're accurate, safe where we don't.

### C. Finish the last metre

Even with relax, the endpoint is 2–3 m short when teach-map tree
inflation surrounds it.

1. **Strip teach-map inflation around the endpoint.** The last WP is
   almost always at the spawn; we know that patch is free at repeat
   time (it's where the robot started). Carve a 2 m disc around the
   endpoint in the static map.
2. **Fall back to dead-reckoning for the final 3 m.** Once within a
   known tolerance of the teach endpoint, disable Nav2 and drive
   straight to the last teach pose.

### D. Operational

1. **Consolidate SKIP -> SKIPPED-RANGE notifications** so the log is
   scannable.
2. Emit a JSON summary at RESULT (reached/skipped/detours/drift
   distribution). Makes cross-run comparison cheap.

## Expected failure modes

- Drift-induced collision. If anchor rate drops in a region, 10 cm
  spec is not enforceable - the robot will clip obstacles by as much as
  the drift magnitude. This is the "real" failure mode of T&R at tight
  clearance and deserves separate treatment (better anchor density /
  online relocalisation).
- **Planner refuses the last WP.** If teach-map tree inflation still
  exceeds 80 cost around the endpoint, the final-WP relax will not help
  and the robot will stop ~2 m short.
