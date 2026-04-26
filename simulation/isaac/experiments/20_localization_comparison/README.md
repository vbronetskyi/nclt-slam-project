# exp 20: Localization Comparison - 3 Routes

Compare ORB-SLAM3 RGBD-only vs RGBD-Inertial (VIO) on all three Husky routes (road, north, south) with a unified IMU pipeline. This provides the localization comparison table for the bachelor thesis.

## результат

| Route | GT distance | RGBD-only ATE | VIO ATE | Notes |
|---|---|---|---|---|
| **Road** | 335 m | 0.405 m | **0.347 m** | VIO 14% better |
| **North** | 467 m | 7.79 m | failed | VIO IMU init breaks on tight forest turns |
| **South** | 396 m | 0.516 m | failed | VIO IMU init breaks on tight forest turns |
