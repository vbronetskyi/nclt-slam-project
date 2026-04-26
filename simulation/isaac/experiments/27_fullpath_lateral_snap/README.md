# exp 27: Full-path lateral + current_idx snap

Exp 26 discovered that the window-based lateral corridor check missed cases
where the watchdog advanced `current_idx` far ahead of the robot's SLAM
position - segments in the window were all beyond the robot, projection
`t` values were <0, and the check silently returned 0 (false negative).

Exp 27 replaces window-based search with full-path search and snaps
`current_idx` back to the closest waypoint on every watchdog escape.

## результат

| Metric | exp 26 | **exp 27** |
|---|---|---|
| Max forward x (GT) | -37.2m (58m) | -20.8m (74m) ¹ |
| Route completion | 35% | 44% ¹ |
| Collisions | 0 | **0** |
| SLAM atlas resets (logged) | 0 | 0 |
| Watchdog escapes | 4 | **10** (kept firing without usefull effect) |
| Cone 1 bypass min | 1.48m | 1.39m x cleanest yet |

¹ The 74m / 44% figure is misleading. See What went wrong below.
