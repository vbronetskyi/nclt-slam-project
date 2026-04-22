# Exp 70 - IMU Fidelity Ablation

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 70_imu_fidelity*


## Мета
Перевірити чи заявлені у дипломі моделі шуму synthetic-IMU (Phidgets 1042)
поводять себе реалістично на повному road-roundtrip маршруті (333 м, 167 WP,
route_memory/road/anchors.json) й визначити чи кожен елемент нашого IMU-pipeline
впливає на дрифт ORB-SLAM3 VIO.

## Дизайн (4-cell orthogonal ablation, Option 2)
| run | accel motion-branch noise | static warmup | мета |
|---|---|---|---|
| baseline | - | - | reference |
| run_A_accel_noise | + Phidgets-1042 noise + bias | - | вплив accel noise |
| run_B_warmup_only | - | +5 s hold | вплив VIO init stabilisation |
| run_AB_both | + | + | комбінований ефект |
| (run_C_tbc_recal) | + | + + recal Tbc | якщо A+B ≠ baseline |

## Маршрут
- Файл: `/tmp/slam_routes.json` -> key `road_nav2` -> `route_memory/road/anchors.json`
- 167 WP, ~333 м, туди й назад
- Start (−90.9, −5.5), End (−92.8, −5.3) - замкнена петля
- **Dense VIO logger стартує ~25 с після початку** (після VIO warmup=200 frames),
  тому перші ~12 м в CSV відсутні. Це не впливає на дрифт-метрики, але дає
  штучний "loop_residual" ~12 м при замірюванні dense_start -> dense_end.

## Метрики
- `drift_max` (m), `drift_mean` (m) - Procrustes 2D + reflection alignment (4-flip search)
- `gt_path` (m), `vio_path` (m), `vio_scale`
- ATE max/mean: pre-turnaround vs post-turnaround vs full
- Порівняння drift-vs-gt_path (не elapsed-time - робот може їхати різною швидкістю)

## Результати

### Baseline
- config: master `run_husky_forest.py`, no fixes
- **gt_path = 321.31 m**, vio_scale = 1.0011 (0.1% scale error)
- **drift_max = 0.74 m**, drift_mean = 0.49 m
- ATE pre-turnaround: max 0.83 m, mean 0.12 m
- ATE post-turnaround: max 0.54 m, mean 0.11 m
- Фінальна точка VIO ≈ anchors start (≈1 м closure)
- Артефакти: `baseline/{vio_pose_dense.csv, traj_gt.csv, metrics.json,
  drift_monitor.log, isaac.log, vio.log, run_husky_forest.py.snapshot}`

### run A - accel noise only
- config: motion-branch `accel_body += N(0, Phidgets σ) + bias`, warmup=0
- **gt_path = 321.1 m**, drift_max = 0.69 m, drift_mean = 0.42 m
- Порівняння drift-vs-gt_path з baseline на ключових точках:

  | gt_path | baseline dmax | run A dmax | Δ |
  |---|---|---|---|
  | 50 m | 0.17 | 0.18 | ≈0 |
  | 100 m | 0.17 | 0.21 | +0.04 |
  | 153 m (turnaround) | 0.43 | 0.42 | ≈0 |
  | 160 m (peak) | **1.06** | **1.02** | −0.04 |
  | 200 m | 0.89 | 0.84 | −0.05 |
  | 250 m | 0.76 | 0.71 | −0.05 |
  | 321 m (finish) | 0.74 | 0.69 | −0.05 |

- **Висновок Run A:** accel-motion-branch noise injection *не є причиною* drift.
  Обидва run'и показують той самий патерн: стабільні ~0.17 m до turnaround ->
  різкий сплеск до ~1 m у точці розвороту -> повільна релаксація до ~0.7 m.
  Різниці у межах run-to-run noise Isaac stochasticity.
- Артефакти: `run_A_accel_noise/{…}` (структура як baseline)
- Плоти: `plots/traj_baseline_vs_runA.png`, `plots/drift_profile_baseline_vs_runA.png`

### run B' - warmup only (pending)
### run A+B - both fixes (pending)

## Запуск
Всі 4 cells діляться одним orchestrator'ом:
```
bash scripts/run_test_drive.sh <out_dir> <husky_script.py> <warmup_s>
```
- Cleanup pkill, Isaac + VIO + TF(GT) + drift_monitor стартують, wall-timeout 1 год
- drift_monitor пише `vio_pose_dense.csv` 10 Hz + лог раз у 10 с

## Відмінність від teach pass (exp 48+, 65)
- Baseline *не* пише depth_map / landmarks (це не teach)
- Drift gate `--max-drift-m 10` (моніторинг, не abort), teach використовує `2` (abort+retry)
- Той самий run_husky_forest, vio_th160.yaml, tf_wall_clock_relay --use-gt
