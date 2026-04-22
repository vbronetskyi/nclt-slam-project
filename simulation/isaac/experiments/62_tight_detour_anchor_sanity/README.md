# exp 62 - Tight Detour + Anchor Sanity

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 62_tight_detour_anchor_sanity*


Drift-focused follow-up to exp 61's failure. Exp 61 inflated the detour
ring (2–5 m -> 4–7 m) which pulled the robot off the teach visual corridor
and let drift grow unbounded (17 m at the critical endpoints). Exp 62
reverts to a tighter ring and adds a sanity gate on incoming anchor
corrections.

## changes vs exp 60 baseline

| Area | Exp 60 | Exp 62 |
|---|---|---|
| Detour ring radii | 2 / 3 / 4 / 5 m | **2 / 2.5 / 3 / 3.5 / 4 m** |
| Anchor sanity gate | - | **reject if `enc_disagree > 2 m` AND `anchor_shift < 0.5 m` AND `std < 0.15`** |
| Precise finisher (turnaround + end) | - | **added - stops pp via empty Path, DR to GT, 0.5 m tol** |

Everything else held constant: accumulator 5 s / 5 m (exp 58/60 stable
values), tf-based REACH, 10 cm clearance spec, Nav2 `robot_radius=0.6`,
`inflation_radius=1.0`, `tolerance=0.3`.

## A1 - tight detour (`2 / 2.5 / 3 / 3.5 / 4 m`)

Five candidate radii, 24 samples each. The closest safe, low-cost
candidate wins. Tight ring keeps the robot inside the 5–8 m wide teach
visual corridor, so the landmark matcher continues to see the same
tree trunks the teach run did.

## A3 - anchor sanity (in `tf_wall_clock_relay_v55.py`)

```
anchor_shift    = |anchor_pose - current_slam_nav|
enc_disagree    = |slam_nav - encoder_dead_reckon|
if enc_disagree > 2 m and anchor_shift < 0.5 m and std < 0.15:
    reject anchor  (matcher latched onto a drifted-map landmark)
```

Motivation from exp 61: once SLAM drifted a few meters, the matcher
started publishing `std = 0.05, shift ≈ 0.2 m` - a very confident "no
correction needed" while encoder was screaming 6+ m of disagreement.
The fused pose then believed the no-correction anchor, locking the
drift in. The gate drops those matches and lets the encoder-vs-SLAM
alpha fallback do its job until the matcher recovers.

## Precise finisher (from exp 61, fixed)

Before the finisher publishes `/cmd_vel`, it publishes an **empty `Path`**
so `pure_pursuit_path_follower` halts its own control loop - exp 61 had
the two loops fighting. Then it runs a simple open-loop GT-referenced
controller:
- spin to face target (0.4 rad/s) when heading error > 0.25 rad
- otherwise `v = min(0.25, 0.4·d)`, `w = clip(0.8·yaw_err)`
- stop when GT distance < 0.5 m
- budget 60 s, one retry on timeout

Critical targets: the max-x teach GT waypoint (turnaround) and the last
teach GT waypoint (return-to-start).

## How to run

```bash
bash scripts/run_exp62_repeat.sh
# results -> results/repeat_run/
```

## success criteria

1. Mean drift ≤ 3 m (exp 60 baseline).
2. Both `PRECISE ARRIVED` events fire - turnaround and end within 0.5 m.
3. No tent/cone body overlap in dense GT (exp 60 had tent −22 cm, cone
   −75 cm; exp 62 must be strictly positive clearance).

## Run 1 results

| | Exp 60 | Exp 61 run 3 | **Exp 62 run 1** |
|---|---|---|---|
| REACHED | 81/95 (85 %) | 83/95 | **88/95 (93 %)** x |
| SKIP | 11 | 9 | **4** x |
| **PRECISE END** | - | timeout 17-20 m | **28 cm ARRIVED** x |
| PRECISE TURNAROUND | - | timeout 17 m | timeout 19 m ✗ |
| SANITY anchor rejects | - | - | 3 |
| Drift mean | 3.11 m | 15+ m | 16.9 m ✗ |
| Drift max | 5.49 m | 18+ m | 99.8 m ✗ |
| Tent body overlap | −22 cm | n/a | **−17 cm** (improvement) |
| Worst cone overlap | −75 cm (at x=+5) | n/a | **−65 cm** (at x=−75) |
| Duration | 32 min | 45 min | 40 min |

### What's fixed
- 88/95 REACHED - best of all runs so far. Tight detour ring kept
  the planner happy; most WPs got a reachable detour target.
- **PRECISE END landed at 28 cm - first time ever.** The empty-Path
  pp_follower silence fix lets the DR finisher actually drive.
- Anchor sanity fires. 3 rejects happened where `std < 0.15, shift
  < 0.5 m, enc_disagree > 2 m` - exactly the self-confirming-drift
  pattern from exp 61.

### What's still broken
- Drift is still catastrophic. Mean 16.9 m, max 99.8 m. The
  sanity gate only drops 3 anchors across a 40 min run - not enough
  to outweigh the normal accumulation of drift during the long
  no-anchor windows.
- **TURNAROUND precise fails** because robot is physically 18 m from
  the target by the time Nav2 "reaches" it in SLAM frame. The
  finisher budget (60 s × 0.25 m/s ≈ 15 m) cannot cover that gap.
- **Physical body overlap still present.** Tent −17 cm, cone −65 cm.
  Detour decisions in SLAM frame are rendered meaningless when drift
  is 5-20 m.

![trajectory](results/repeat_run/trajectory.png)

## next

1. **Make turnaround precise fire earlier.** Currently triggered on
   SLAM REACH (d<3 m in SLAM), but robot may be 18 m away in GT.
   Option: also check GT distance before declaring Nav2 done, loop
   Nav2 goals until GT distance is within finisher budget.
2. **Attack drift at the source.** The sanity gate is too narrow
   (3 rejects/run). Need global matcher fallback when `anchor_age >
   30 s` - search all teach landmarks by descriptor, ignore current
   VIO pose. Would recover from "kidnapped robot" in mid-route.
3. **Known-obstacle check in SLAM frame.** Transform cone/tent
   coords from GT to current-SLAM via the last good anchor's
   reported shift, so clearance check stays consistent with the
   planner.
