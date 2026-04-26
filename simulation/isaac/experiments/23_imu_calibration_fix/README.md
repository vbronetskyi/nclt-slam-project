# exp 23: IMU calibration fix - VIO finally works on road (ATE 0.089m)

Find and fix all IMU/Tbc calibration errors that were breaking VIO on forest routes (and silently degrading it on road).

## результат

| Configuration | ATE | Scale | Resets/run | VIBA 2 success |
|---|---|---|---|---|
| Old broken (swapped axes, wrong Tbc) | 0.347 m | 0.99 | 0-2 | 1/6 runs |
| **Corrected (UBR->FLU + Tbc)** | **0.089 m** | **0.996** | **0** | **5/5 runs** |
