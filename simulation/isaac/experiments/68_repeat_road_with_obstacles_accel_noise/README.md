# exp 68 - Road repeat WITH obstacles + accel-noise IMU

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 68_repeat_road_with_obstacles_accel_noise*


## Мета
Exp 59 Pareto-оптимальний pipeline на road маршруті з фізично коректною IMU
(accel noise у motion branch, exp 70 Run A), тепер **з повним набором
obstacles** (17 cones + 1 tent) на шляху outbound і їх автоматичним
видаленням після turnaround - відтворити класичний T&R detour-heavy
сценарій exp 59 на road route.

## Налаштування обʼєктів
`spawn_obstacles.py road` (1 m spacing, halved):
| Barrier | x | y range | cones | bypass |
|---|---|---|---|---|
| BARRIER 1 | −50 | [−8.0, −3.0] | **6** | north y>−2 (3 m free) |
| BARRIER 2 | +15 | [−1.0, +4.0] | **6** | south y<−1.5 (4 m) |
| BARRIER 3 | +45 | [−3.0, +1.0] | **5** | south y<−3.5 або north y>+1.5 |
| Tent | (−20, 0) | 2.0 × 1.8 m footprint | 1 | (на route, detour обов'язковий) |

**Total: 17 cones + 1 tent. Removed at x ≥ 70 (turnaround)** через
`turnaround_supervisor.py --turnaround-x 70.0 --past-margin 2.0`.

## Вхідні артефакти
- exp 66 teach: `teach_map.yaml`, `landmarks.pkl`, `vio_pose_dense.csv`
- exp 66 `run_husky_forest.py` (accel noise)
- exp 59 pipeline (tf_v55, matcher, Nav2 planner_only, PP, hybrid goal sender, supervisor)

## Фікс перед запуском
`export PYTHONPATH=/workspace/simulation/isaac/scripts:...` у orchestrator
перед Isaac - інакше `from spawn_obstacles import ...` падав
`ModuleNotFoundError`, бо Isaac-python бачить тільки директорію run_husky_forest.py.

## Результати

| Метрика | Значення |
|---|---|
| **REACHED** | **77 / 80 (96.25 %)** |
| SKIP | 3 |
| DETOUR triggered | 8 |
| Projections | 111 (costmap-based WP projections) |
| Duration | 1357 s (≈ 23 min) |
| gt_path | 388.3 m (+23 m від exp 67 за рахунок detour обʼїздів) |
| **drift_max (nav vs GT)** | **14.28 m** <- anchor misfire під час BARRIER 2/tent detour |
| drift_mean | 2.06 m |
| samples drift > 3 m | 72 / 1115 (6.5 %) |
| Anchor publishes | 144 / 1115 (12.9 %) |
| Supervisor FIRED | x (GT x passed 70, decreased 2 m) |

### Порівняння з exp 67 (road, без обʼєктів) та exp 59 (south, з обʼєктами)

| | exp 59 (south + obstacles) | exp 67 (road, no obstacles) | **exp 68 (road + obstacles)** |
|---|---|---|---|
| REACHED | 79 / 95 (83 %) | 80 / 80 (100 %) | **77 / 80 (96 %)** |
| SKIP | 10 | 0 | 3 |
| DETOUR | 18 | 3 | 8 |
| drift_max | 3.64 m | 1.48 m | **14.28 m** |
| drift_mean | 1.67 m | 0.70 m | **2.06 m** |
| duration | 35 min | 14 min | **23 min** |
| anchor rate | 15 % | 22.6 % | 12.9 % |

**Ключове спостереження:**
- Reach-rate впав 100 % -> 96 % через 3 SKIP у зонах BARRIER 2/3 (planner не знайшов шлях до original WP після 5× спроб)
- **Drift_max стрибнув до 14 m у зоні tent + BARRIER 2 (dist=87–128 m)** - ROOT CAUSE:
  roundtrip landmarks.pkl містить обидва напрями (40 outbound + 11 return
  anchors). Під час outbound detour FOV змінюється, matcher серед 51 anchor
  обирає «найсхожіший», і ним стає return-leg anchor (той самий x, yaw
  ≈ +180°). PnP дає позу на симетричному боці дороги (nav y=−16 замість
  GT y=−3). `HEADING_TOL_DEG=90°` не вирізав цей випадок. Після turnaround
  анкори з правильного напряму підхоплюються і drift повертається до 0.5 m.
  -> **Mitigation у exp 69**: split landmarks.pkl outbound/return, swap
  сету через supervisor при turnaround.
- Duration 1357 s vs 830 s exp 67 - detour + SKIP recovery додає ~50 % часу.
- Supervisor спрацював коректно: обʼєкти зникли при повороті назад, costmap очистився за ~10 с через `clearing=True` depth raycast.

### Плоти
- [results/repeat_run/trajectory.png](results/repeat_run/trajectory.png) - GT + SLAM-nav на мапі з обʼєктами
- [results/repeat_run/drift_profile.png](results/repeat_run/drift_profile.png) - drift-vs-distance з misfire zone

## pipeline
1. **Isaac** (з obstacles) + `run_husky_forest.py` з accel noise (exp 66 копія)
2. **ORB-SLAM3 RGB-D-I** VIO (vio_th160.yaml)
3. Phase 1 - GT-tf warmup (200 frames)
- Phase 2 - swap to SLAM-tf v55 + matcher + Nav2 + PP + supervisor
5. visual_landmark_matcher -> `/anchor_correction`
6. **Nav2 planner_only** (strict: robot_radius 0.7, inflation 1.5, tolerance 0.3)
7. **pure_pursuit_path_follower** -> `/cmd_vel`
8. turnaround_supervisor -> `/tmp/isaac_remove_obstacles.txt` -> Isaac `remove_obstacles(stage)`
- plan_logger - > `plans/*.json`
10. send_goals_hybrid - teach vio trajectory @ 4 m spacing + lookahead skip + detour (4–7 m ring) + SLAM-frame REACH check

## Запуск
```
bash scripts/run_exp68_repeat.sh
```

## Висновок
Exp 59 pipeline **повноцінно працює на road+obstacles+accel-noise** з
reach-rate 96 % та перевершуючи exp 59 (83 %). 3 SKIP на vertical barriers
де planner іноді не знаходить path. Anchor misfire створює 14 m transient
drift у detour-heavy зоні, але система відновлюється self-correction після
turnaround. Thesis baseline витримує real-world обʼєкти на карті.

## Наступні кроки
- (Optional) exp 69: tune anchor matcher (reject matches with large translation shift) щоб уникнути 14 m misfire
- (Optional) повернення SKIP -> REACH через retry або нижчий timeout
