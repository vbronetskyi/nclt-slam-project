# exp 21: North Drift Analysis - zigzag init impact

Investigate why north RGBD-only ATE was 7.79m in exp 20 (vs old 1.93m baseline), and verify VIO failure mode + IMU configuration.

## результат

| Configuration | ATE | Max error | Notes |
|---|---|---|---|
| **WITH zigzag init** (default) | **7.79 m** | 26.5 m | High drift in first 60 s |
| **WITHOUT zigzag init** (skip first 200 frames) | **1.72 m** | 5.4 m | Stable throught route |
