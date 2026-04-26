# exp 33: VIO SLAM Tuning (ORB-SLAM3 RGBD-Inertial)

Налаштувати ORB-SLAM3 VIO (RGBD + IMU) для south forest route і порівняти
з RGBD-only SLAM.

## результат

| Config | ThDepth | Max depth | Tracked | Scale | ATE |
|---|---|---|---|---|---|
| th80_default | 80 | 10m | **73%** | 0.067 | 32.38m |
| th100_default | 100 | 12.5m | 72% | 0.018 | 33.42m |
| th100_noisy | 100 | 12.5m | 48% | 0.086 | 16.47m |
| th100_balanced | 100 | 12.5m | 48% | 0.037 | 25.70m |
| th100_3kfeat | 100 | 12.5m | 48% | 0.025 | 22.13m |
