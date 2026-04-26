# exp 22: VIO Warmup for Forest Routes

After confirming VIO offline works on road (exp 18: ATE 0.116m), but fails on forest routes (exp 20: 60+ resets), try prepending a warmup motion to forest recordings so ORB-SLAM3 IMU init has time to converge before sharp forest turns.

## результат

| Recording | RGBD-only ATE | VIO ATE (full route) | VIO ATE (warmup only) |
|---|---|---|---|
| Road (exp 20, 335m) | 0.272 m | 0.347 m | n/a |
| North (exp 22, 467m) | 1.86 m | failed (24 resets) | **0.80 m** (44m segment) |
