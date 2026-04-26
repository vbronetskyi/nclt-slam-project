# exp 26: Forward regression watchdog + tighter recovery

Exp 25 run 4 revealed a recurring failure: after RGBD-only Nav2 bypasses
obstacle 1 (cone group at x=-50), the robot enters a Nav2 recovery loop,
regresses backwards in GT, and can't break out. Depth-seen bushes block the
replanner's attempt to return to the road.

This experiment adds four targeted fixes to break out of recovery loops:

| Fix | Where | What |

## результат

| Metric | Run 25/4 | **Run 26** |
|---|---|---|
| Max forward x (GT) | -45.8m (49m) | **-37.2m (58m)** <- +9m (+18%) |
| Route completion | 29% | **35%** |
| Collisions | 0 | **0** |
| SLAM atlas resets | 0 | **0** |
| Regression watchdog escapes | - | 4 (all after cone 1) |
| Cone 1 bypass min distance | 1.70m | **1.48m** |

New best for road outbound with obstacles. Cone 1 bypassed cleanly.
