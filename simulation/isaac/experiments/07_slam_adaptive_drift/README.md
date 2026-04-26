# exp 07: SLAM + adaptive drift monitor (best SLAM result before VIO)

Fix exp 06's drift monitor oscillation with **adaptive thresholds and forward-only waypoints**.

## результат

- best pre-VIO SLAM

**138 m, 76 % route, all 4 obstacle groups bypassed.** This was the best SLAM result before the SLAM-frame aproach (exp 09-14).

| Metric | exp 07 | exp 06 (drift v1) | exp 02 (raw SLAM) |
|---|---|---|---|
| Distance | **138 m** | 123 m | 106 m |
| Route % | 76 % | 67 % | 58 % |
| Obstacles bypassed | 4/4 | 3/4 | 1/4 |
