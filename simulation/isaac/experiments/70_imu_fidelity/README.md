# Exp 70 - IMU Fidelity Ablation

Перевірити чи заявлені у дипломі моделі шуму synthetic-IMU (Phidgets 1042)
поводять себе реалістично на повному road-roundtrip маршруті (333 м, 167 WP,
route_memory/road/anchors.json) й визначити чи кожен елемент нашого IMU-pipeline
впливає на дрифт ORB-SLAM3 VIO.

## результат

| run | accel motion-branch noise | static warmup | мета |
|---|---|---|---|
| baseline | - | - | reference |
| run_A_accel_noise | + Phidgets-1042 noise + bias | - | вплив accel noise |
| run_B_warmup_only | - | +5 s hold | вплив VIO init stabilisation |
| run_AB_both | + | + | комбінований ефект |
| (run_C_tbc_recal) | + | + + recal Tbc | якщо A+B ≠ baseline |
