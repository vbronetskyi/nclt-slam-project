# exp 16: VIO mapping with filter=60 IMU - FAILED (different bug)

Test ORB-SLAM3 RGBD-Inertial mapping with the cleaner IMU from exp 15
(filter_size=60, noise std matches real Phidgets Spatial 1042).

## результат

: FAILED

281 of 285 frames lost. SLAM never initialized tracking. Same failure mode
as previous VIO attempts (exp 5-7), even though IMU noise is now Phidgets-level.
