# exp 48: VIO Live Round-Trip (VIO vs RGB-D only)

## результат

Full round-trip 392м (outbound -> turnaround -> return).

| Method | ATE RMSE | Scale | Frames | Notes |
|---|---|---|---|---|
| **VIO (RGB-D-Inertial)** | **0.491m** | 1.001 | 5588 (0 lost) | live, synthetic IMU 200Hz |
| RGB-D only (no IMU) | 1.984m | 1.022 | 5543 (1 reset) | offline rgbd_tum |

IMU дає 4* кращу точність. Обидва успішно прошли 392м маршруту.
