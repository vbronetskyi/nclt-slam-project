# exp 17: Offline RGBD-Inertial SLAM with filter=60 IMU

Test ORB-SLAM3 RGBD-Inertial offline mode (avoiding live VIO sync bugs from exp 16) with the cleaner IMU from exp 15 (filter=60, Phidgets-level noise).

Hypothesis: cleaner IMU + offline = working VIO with ATE close to RGBD-only baseline (0.49m).

## результат

| Config | Maps | Fail tracks | ATE | Scale |
|---|---|---|---|---|
| v3 Variant B (1500 frames) | 3 | 55 | 22.0m | 0.150 |
| v1 Identity (1500 frames) | 2 | 1 | 8.5m | 0.078 |
| RGBD-only baseline | - | - | 0.49m | 1.0 |
