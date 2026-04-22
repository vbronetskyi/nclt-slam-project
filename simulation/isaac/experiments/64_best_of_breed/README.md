# Exp 64 - Best-of-Breed (Exp 59 that finishes)

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 64_best_of_breed*


Minimal-delta experiment: take exp 59 (#1 in composite ranking) and add
two proven-working mechanisms from later experiments, with nothing else
changed.

## motivation

Post-hoc analysis showed:
- Exp 59 has the best composite score, best TRACK p50 (1.60 m), best
  loop closure (5.95 m) and best smoothness - Pareto-optimal on most
  dimensions.
- BUT exp 59 stops 5-6 m short of the teach endpoint because its
  last WP gets detoured (teach-map tree inflation forces a safe target
  a few meters from the true end).
- Exp 62's **precise finisher** already solved this: it closed to 28 cm
  in GT. But exp 62 broke TRACK elsewhere (16.9 m mean drift) due to
  other changes.
- Exp 60's STOP fix (empty Path + zero Twist after RESULT) prevents
  the robot from spinning in place when no more goals are coming.

Strategy: take exp 59 *unchanged*, bolt on just the STOP and
precise-END finisher. Expected: all of exp 59's strengths + close loop.

## surgical changes from exp 59

Three small patches, nothing else:

1. **`send_goals_hybrid.py`** - new `_precise_approach()` method. Called
   only for the last WP (`i == n_wps - 1`) after normal Nav2 follow.
   Publishes empty `Path` to silence pp_follower, then drives `/cmd_vel`
   directly using `/tmp/isaac_pose.txt` (GT) until within 0.5 m of
   `original_wps[-1]`. Budget 60 s, one retry on timeout.

2. `send_goals_hybrid.py` - after `RESULT:` log line, publish empty
   `Path` + zero `Twist` three times so pp_follower halts cleanly.

3. **`pure_pursuit_path_follower.py`** - treat `len(msg.poses) == 0`
   as STOP: drop the active plan and publish zero Twist.

That's it. Detour ring stays at 4-7 m, Nav2 `robot_radius=0.7`,
`inflation_radius=1.5`, `tolerance=0.3`, accumulator 5 s / 5 m - all
exp 59's values.

## Expected vs exp 59

| Metric | Exp 59 | Exp 64 (target) |
|---|---|---|
| TRACK p50 | 1.60 m | 1.60 m (unchanged) |
| ARRIVAL p50 | 3.68 m | 3.68 m (unchanged) |
| **Loop closure** | **5.95 m** | **≤ 1 m** x |
| Tent clearance | −22 cm | −22 cm (unchanged) |
| Cone clearance | −75 cm | −75 cm (unchanged) |
| Path ratio | 1.41× | 1.41× |
| Runtime | 10 min | 10 min |
| Composite (outdoor) | 0.712 | > 0.75 (projected) |

## explicitly NOT changed

- Detour ring radii (4/5/6/7 m) - tighter ring in exp 60 gave worse
  loop closure
- Nav2 costmap params - exp 60's aggressive tightening made no
  measurable clearance difference but hurt other metrics
- Accumulator thresholds (5 s silence, 5 m min-dist) - exp 61 showed
  aggressive accumulator creates drift runaway
- No global relocalisation (exp 63 showed it destabilises anything
  that works)
- No anchor sanity gate (exp 62's gate fired 3 times in 40 min - too
  rare to matter, not worth risk)

## How to run

```bash
bash scripts/run_exp64_repeat.sh
# Results -> results/repeat_run/
```

## Success criteria

1. `PRECISE ARRIVED END` fires with d < 0.5 m of `original_wps[-1]`.
2. Loop residual ≤ 1.5 m (i.e. close to teach's own 5.41 m floor plus
   the finisher's 0.5 m tolerance).
3. All other metrics within ±10 % of exp 59's numbers - this is a
   minimal-delta experiment, so the only allowable change is endpoint.
4. Composite score strictly higher than exp 59's 0.712 (since `completion`
   sub-score will lift via smaller loop residual).
