# exp 05: SLAM + IMU gyroscope yaw fusion (without VIO)

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 05_slam_imu_gyro_yaw*


## Goal

Use the IMU gyroscope (which is cheap and has very low noise on the z axis) to provide a stable yaw reference. SLAM RGBD-only handles position; IMU handles yaw. **No ORB-SLAM3 VIO** - gyro integrated directly in the relay.

## why this is needed

In exp 02-03, lateral drift is ~5-10 m. The root cause is yaw drift: SLAM's yaw estimate accumulates error frame-to-frame, and a small yaw error over a 100 m trajectory becomes a large lateral position error. IMU gyroscope is much more stable than visual yaw.

## Setup

In `tf_wall_clock_relay.py`:

```python
# IMU at 200 Hz, read gz (yaw rate in FLU frame)
filtered_gz = LPF(raw_gz, alpha=0.3)
if abs(filtered_gz) < 0.08: filtered_gz = 0  # deadzone (PhysX noise)
imu_yaw += filtered_gz * dt

# SLAM yaw correction (slow)
imu_yaw += SLAM_YAW_ALPHA * (slam_yaw - imu_yaw)  # alpha = 0.15
```

Position from SLAM directly. Yaw from IMU + slow SLAM correction.

## result

**102 m, 56 % route, cones bypassed.**

| Metric | Value |
|---|---|
| Position error on straights | **1.2 m** (vs 8 m without gyro) |
| Lost frames | 0.1 % |
| Stuck at | x=-23, y=10 (same as exp 02-03) |

## what we learned

1. **Gyro yaw fusion works on straights** - error reduced from 8 m to 1.2 m. The IMU keeps yaw stable between SLAM updates, preventing drift accumulation.
2. Doesn't help after maneuvers. During obstacle bypass, the robot makes large yaw changes. SLAM yaw becomes unreliable for a few seconds (relocalization). The slow `SLAM_YAW_ALPHA = 0.15` correction can't pull the gyro yaw back fast enough -> `imu_yaw` drifted to 6.31 rad after maneuvers.
3. **Wrap-around bug** - at large angles (~±π), SLAM yaw conversion is unstable. The slow correction can pull yaw the wrong way around.

Stuck at the same x=-23 location as exp 02-03 - the obstacle bypass takes the robot off the mapped trajectory and into territory with no visual reference.

## conclusion

IMU gyro is a real improvement for **straight-line drift**, but doesn't fix the **post-maneuver drift** which is the actual blocker for full route completion. This finding leads to drift monitor approaches (exp 06-07).

## files

- `scripts/tf_wall_clock_relay.py` - adds IMU subscription, gyro integration, complementary yaw filter
- `results/trajectory_plot.png`
