# exp 05: SLAM + IMU gyroscope yaw fusion (without VIO)

Use the IMU gyroscope (which is cheap and has very low noise on the z axis) to provide a stable yaw reference. SLAM RGBD-only handles position; IMU handles yaw. **No ORB-SLAM3 VIO** - gyro integrated directly in the relay.

## результат

**102 m, 56 % route, cones bypassed.**

| Metric | Value |
|---|---|
| Position error on straights | **1.2 m** (vs 8 m without gyro) |
| Lost frames | 0.1 % |
| Stuck at | x=-23, y=10 (same as exp 02-03) |
