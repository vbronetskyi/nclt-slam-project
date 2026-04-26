# exp 69 - Road repeat, split outbound/return landmarks + accel-noise IMU

Усунути 14 m anchor-misfire spike, виявлений у exp 68.

## результат

| dist | nav (SLAM belief) | GT (truth) | err |
|---|---|---|---|
| 95 m | (0.9, **−1.4**) | (0.7, **+3.5**) | 4.9 m |
| 112 m | (6.9, **−10.4**) | (9.3, −1.3) | 9.5 m |
| 126 m | (15.9, **−16.4**) | (20.4, **−3.0**) | **14.1 m** |
