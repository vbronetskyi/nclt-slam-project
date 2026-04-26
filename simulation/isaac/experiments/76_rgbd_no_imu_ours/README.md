# Exp 76 - Our full T&R pipeline, but ORB-SLAM3 in pure RGB-D (no IMU)

## результат

| Component | our pipeline | exp 76 |
|---|---|---|
| ORB-SLAM3 binary | `./Examples/RGB-D-Inertial/rgbd_inertial_live` | `./Examples/RGB-D/rgbd_live` |
| Config YAML | `vio_th160.yaml` (IMU noise + Tbc) | `rgbd_th160.yaml` (no IMU block) |
| `--synthetic-imu` CLI flag | on | off |
| Everything else | - | **unchanged** |
