# Exp 76 - Our full T&R pipeline, but ORB-SLAM3 in pure RGB-D (no IMU)

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 76_rgbd_no_imu_ours*


Same as our per-route `routes/09_se_ne/repeat` pipeline, but ORB-SLAM3
runs in **RGB-D only** mode (no IMU inertial tracking).  Everything else
(matcher, tf_relay, Nav2 planner + our send_goals_hybrid + pure_pursuit,
turnaround_supervisor, detour ring, final-WP policy) is identical.

## Why

Tests whether the IMU is load-bearing for our T&R pipeline.
pretty sure IMU matters in fast turnarounds where the camera blurs, but this
experiment puts a number on it.  The IMU is
used by ORB-SLAM3's inertial back-end to keep VIO pose accurate accross
featureless or motion-blurred frames.  Without it, ORB-SLAM3 falls back
to pure RGB-D tracking - works when enough visual features are
available; breaks on visual dropouts.

## What changed vs our custom pipeline

| Component | our pipeline | exp 76 |
|---|---|---|
| ORB-SLAM3 binary | `./Examples/RGB-D-Inertial/rgbd_inertial_live` | `./Examples/RGB-D/rgbd_live` |
| Config YAML | `vio_th160.yaml` (IMU noise + Tbc) | `rgbd_th160.yaml` (no IMU block) |
| `--synthetic-imu` CLI flag | on | off |
| Everything else | - | **unchanged** |

## Expected vs ours

| Metric | Our RGB-D-I | Exp 76 RGB-D only |
|---|---|---|
| Reached / total | 36 / 36 | TBD |
| Duration | 689 s | TBD |
| Final WP reached | x | TBD |
| Teach drift (mean / max) | 0.40 / 0.64 m (teach) | TBD |
| SLAM frames lost | 0 | TBD (more expected) |

## How to run

```bash
ROS_DOMAIN_ID=89 bash scripts/run_rgbd.sh
# outputs in results/run_09/
```
