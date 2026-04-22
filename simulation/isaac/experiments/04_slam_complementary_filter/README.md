# exp 04: SLAM + cmd_vel odometry complementary filter

## Goal

Reduce SLAM drift between updates by fusing it with `cmd_vel` integration (dead reckoning). Idea: SLAM corrects long-term, odometry smooths short-term frame-to-frame jitter.

## Setup

In `tf_wall_clock_relay.py`:

```python
# Subscribe to /cmd_vel, integrate as dead reckoning
odom_x += cmd.linear.x * dt * cos(odom_yaw)
odom_y += cmd.linear.x * dt * sin(odom_yaw)
odom_yaw += cmd.angular.z * dt

# complementary filter: blend SLAM + odom
fused_x = alpha * slam_x + (1 - alpha) * (fused_x + odom_dx)
```

Two `alpha` values tested:
- `alpha = 0.05` - odometry dominates
- `alpha = 0.3` - SLAM dominates

## results

| alpha | Y drift | route % | what happened |
|---|---|---|---|
| 0.05 (odom-heavy) | **18.6 m** | 57 % | worse than raw SLAM |
| 0.3 (SLAM-heavy) | 9 m | 58 % | filter just smooths jitter |

## What we learned

1. `cmd_vel` ≠ actual motion on skid-steer. Husky's actual velocity depends on ground friction, wheel slip, terrain. The commanded velocity diverges from real velocity by ~10-20 % regularly. With `alpha=0.05`, the noisy odometry pulls the fused position away from the (less noisy) SLAM estimate.
2. `alpha=0.3` makes the filter useless. It tracks SLAM closely (because SLAM dominates) and only helps with frame-to-frame jitter - not the underlying drift.
3. **Dead reckoning doesn't help with SLAM drift. SLAM drift is in the lateral** direction. `cmd_vel` integration drifts in the same direction because it inherits errors from the same source (wrong yaw estimate).

## Conclusion

Skid-steer `cmd_vel` is too noisy to use as a complementary signal. On real hardware, wheel encoders (not `cmd_vel`) would give cleaner odometry but still suffer from wheel slip on uneven terrain. **Filter helps with jitter but not drift.**

This experiment is the reason later approaches don't rely on cmd_vel integration as a primary signal - instead they use IMU gyro for yaw (exp 05) or work in SLAM coordinate frame (exp 09).

## Files

- `scripts/tf_wall_clock_relay.py` - adds cmd_vel subscription and complementary filter
- `results/trajectory_plot.png`
