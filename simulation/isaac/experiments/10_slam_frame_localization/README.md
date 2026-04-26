# exp 10: SLAM frame + localization mode (failed combination)

Combine the two best approaches: SLAM frame navigation (exp 09, 141 m) + atlas localization (exp 03). Hypothesis: localization mode -> drift pattern matches the atlas -> waypoints from the atlas align better with live pose.

## результат

- failed

**+-50 m, 28 % route.** Worse than exp 02-09.

Multiple sub-experiments:
- 10a (timeout=15s): cascading skips, route abandoned at 50 m
- 10b (timeout=45s): better, but still stuck at 80 m
- 10c (timeout=90s): hit cone obstacles, drift > 10 m
