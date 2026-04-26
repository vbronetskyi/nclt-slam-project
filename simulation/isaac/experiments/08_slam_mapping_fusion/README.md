# exp 08: SLAM mapping mode + full sensor fusion

Combine three signals: SLAM position + wheel odometry + IMU gyro compass. Use mapping mode instead of localization to avoid the unknown features after obstacle bypass problem from exp 03.

Hypothesis: in mapping mode, every visible feature is added to the map -> no problem if robot leaves the original mapped trajectory.

## результат

- WORST SLAM result

**47 m, 28 % route.** Worse than all previous SLAM attempts.
