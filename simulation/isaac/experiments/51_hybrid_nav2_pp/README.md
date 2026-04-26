# Exp 51 - Hybrid Nav2 planner + Pure Pursuit follower with VIO localization

## результат

| version | change | WPs reached | max dist | verdict |
|---|---|---|---|---|
| **v1** (this dir, root `config/` + `scripts/`) | base hybrid setup | 0 | 51 m | fails at first turn |
| **v2_scale_correction** | encoder-distance scale correction in tf_relay | - (dropped) | - | papered over symptom; offline data showed rolling scale is too noisy to improve blended output. Diagnostic scripts here led to the real root cause. |
| **v2_imu_fix** | synthetic-IMU standstill detection in `run_husky_forest.py` | 32 | 192 m | first 150 m <2 m err; catastrophic BAD LOOP accept at 150 m |
| **v3_no_loop** | v2_imu_fix + `loopClosing: 0` + patched `LoopClosing.cc::Run()` early-exit | **33** | 174 m | first 100 m <1.5 m err; gradual drift past 125 m (not loop-closure related) |
| **v4_anomaly_guard** | v3 + `SlamAnomalyGuard` in tf_relay (per-tick displacement > 0.8 m -> freeze SLAM) | 33 | 428 m | guard never fired - the real problem isn't single-tick jumps, it's **gradual scale drift** (distributed across many ticks). err grew 0.3 m -> 28 m across 0-300 m. |
