# Exp 55 - visual teach-and-repeat landmark matching for drift-free VIO

(from exp 53 analysis)

Exp 53 run 2 reached 37/94 WPs, 539 m before SLAM drift ran away (32 m err
by 400 m) in feature-sparse forest. Cause: only 2 VIBA events total ->
IMU bias walked freely over 5 400 s -> pure yaw-bias signature in position
drift.

Exp 54 experiments with adaptive SLAM/encoder blend made things worse

## результат

- run 3 (scene-fixed, 0.8 m/s, v54 fallback + wedge-recovery)

| Metric                   | Value |
|--------------------------|-------|
| Distance driven          | **477 m** |
| Reached max x            | 83 (past turnaround x=60) |
| WPs reached              | 19 / 94 |
| WPs timeout              | 4 |
| WPs skipped (plan failed)| 20 |
| Wedge recoveries         | 23 |

