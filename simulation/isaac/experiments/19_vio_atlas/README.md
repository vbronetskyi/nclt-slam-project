# exp 19: VIO Atlas - partial success

Build a reusable VIO atlas offline, then use it in live localization mode for navigation. This would let us:
1. Build atlas once (offline, deterministic)
2. Use it many times during navigation (live, fast)
3. Get stable VIO localization without per-run init issues

## результат

We have a **working VIO mapping pipeline** (offline, ATE 0.469m), but **VIO live localization with pre-built atlas doesn't work as expected** - it creates a fresh map instead of relocalizing.

For navigation, the practical options remain:
1. **Use exp 13/14 SLAM-frame approach** (RGBD-only mapping mode + waypoints extracted from same session) - current best at 145m
- Run full VIO mapping inline during navigation - would work but no benefit over RGBD-only since waypoints are still per-run
3. **Use RGBD-only atlas for relocalization** (which works) and use IMU only for gyro yaw fusion in relay...
