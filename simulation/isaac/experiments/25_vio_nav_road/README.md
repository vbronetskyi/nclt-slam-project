# exp 25: VIO round-trip limitation + SLAM tracking loss in Nav2 recovery

Run Nav2 with obstacles on road route using the best localization. After exp 23's
claim of VIO road ATE 0.089m, try VIO for mapping + live localization for navigation.

## результат

| Segment | ATE | Scale | Stability (runs) |
|---|---|---|---|
| Outbound only (head -1500) | **0.14-0.15m** | 0.994-0.997 | 3/3 consistent |
| Full round-trip (3381 frames) | 1.8-33m | 0.02-1.0 | 8/8 inconsistent |
