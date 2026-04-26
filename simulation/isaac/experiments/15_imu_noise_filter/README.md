# exp 15: PhysX IMU noise reduction for ORB-SLAM3 VIO

Find a way to get clean IMU readings from PhysX in Isaac Sim so ORB-SLAM3 RGBD-Inertial mode can work. Previous attempts at VIO failed because IMU readings during motion had massive noise from contact solver micro-oscillations.

## результат

| Filter size | Window | Stationary acc | Moving acc | Stationary gyro | Moving gyro |
|---|---|---|---|---|---|
| 1 (no filter) | 5ms | 0.009 | **3.20** | 0.0002 | **0.10** |
| 20 (production) | 100ms | 0.0001 | 0.218 | 0.0001 | 0.064 |
| 40 | 200ms | 0.0000 | 0.074 | 0.0000 | 0.037 |
| **60** | **300ms** | **0.0000** | **0.050** | **0.0000** | **0.022** |
| 100 | 500ms | 0.0000 | 0.049 | 0.0000 | 0.014 |
