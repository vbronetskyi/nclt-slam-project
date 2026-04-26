# exp 04: SLAM + cmd_vel odometry complementary filter

Reduce SLAM drift between updates by fusing it with `cmd_vel` integration (dead reckoning). Idea: SLAM corrects long-term, odometry smooths short-term frame-to-frame jitter.

## результат

| alpha | Y drift | route % | what happened |
|---|---|---|---|
| 0.05 (odom-heavy) | **18.6 m** | 57 % | worse than raw SLAM |
| 0.3 (SLAM-heavy) | 9 m | 58 % | filter just smooths jitter |
