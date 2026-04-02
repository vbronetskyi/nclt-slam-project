# Exp 61 - Denser Anchors + Precise Endpoint Arrival

User-directed follow-up to exp 60. Two priorities:

1. **Reduce drift.** Exp 60 mean drift was 3.11 m; no-anchor windows
   reached 15 min. Aggressive clearance (10 cm spec) is meaningless at
   that drift. Tighten the continuous-accumulation matcher so anchors
   appear more often along the route.
2. **Precise arrival at the two critical points.** The turnaround
   (max-x point in teach GT) and the return endpoint (first/last teach
   GT point) are **non-skippable** and must be reached within **0.5 m**
   of the teach GT coordinate.

## Run 1 failed -> run 2 policy change

Run 1 observations:
- **Drift runaway.** 2 s / 2 m accumulator was over-aggressive: it
  recorded landmarks from already-drifted poses, then re-matched to
  them, reinforcing drift instead of correcting. Drift hit 5.4 m by
  dist = 125 m (exp 60 was < 1 m at the same distance).
- Physical tent hit around WP 16. The goal sender correctly
  flagged WP 16 as `near_tent` and DETOURed to 4.5 m away - but with
  ~3 m drift the detour was physically on the tent. User feedback:
  "should have been skipped; let Nav2 go around".

Run 2 changes:
- Accumulator backed off to 3 s / 3 m.
- `near_known_obstacle` -> SKIP (not DETOUR). The next WP's Nav2
  plan handles the go-around naturally via costmap inflation.
- Costmap-only `cost ≥ 60` WPs still use DETOUR (those are real
  teach-map tree phantoms without a physical body to hit).

## Changes vs exp 60

| Area | Exp 60 | Exp 61 |
|---|---|---|
| Accumulator silence trigger | 5.0 s | **2.0 s** |
| Accumulator min-dist to existing landmark | 5.0 m | **2.0 m** |
| Detour ring radii | 2 / 3 / 4 / 5 m | **4 / 5 / 6 / 7 m** (reverted) |
| Critical WPs (turnaround + end) | treated as normal | **non-skippable, precise finisher** |
| Precise finisher tolerance | - | **0.5 m body-edge to teach GT** |

Everything else carried over from exp 60: tf-based REACH, lookahead
detour, hardcoded known-obstacle check, clean `STOP` after RESULT,
final-WP detour-cost relax.

## precise finisher

After Nav2 brings the robot within its normal tolerance of the critical
WP, the goal sender switches to an open-loop GT-referenced controller:

```
read /tmp/isaac_pose.txt  -> gt_x, gt_y, yaw
compute heading error to target
if |yaw_err| > 0.25 rad  -> spin in place at 0.4 rad/s
else                     -> v = min(0.25, 0.4·d),  w = clip(0.8·yaw_err)
stop when d < 0.5 m
```

Budget 60 s per attempt; one retry on timeout. The controller publishes
`/cmd_vel` directly - Nav2 and pp_follower are bypassed for the final
metre.

Why GT (in sim only): this experiment demonstrates the precise-
arrival *capability*. In a real deployment the same finisher would use
the visual landmark matcher's confirmed fix as its pose source - the
hand-off logic is identical; only the sensor behind the two numbers
changes.

## How to run

```bash
bash scripts/run_exp61_repeat.sh
# Results -> results/repeat_run/
```

## Open items deferred to exp 62

- Drift-aware clearance. `KNOWN_CLEARANCE_M = 0.1 + 1.5 · drift_est`.
  Needs `enc_err` published as a ROS topic from `tf_wall_clock_relay_v55`.
- **Teach-map carve.** 2 m free disc in the static map around the spawn
  and turnaround cells so the planner doesn't detour the endpoint into
  tree inflation.
- Anchor-confirmed endpoint. Replace GT readout in the precise
  finisher with an anchor-triggered hand-off (pose from PnP fix).
