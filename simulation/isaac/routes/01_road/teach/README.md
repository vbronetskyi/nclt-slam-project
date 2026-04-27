# Exp 66 - Road teach with accel-noise IMU

*[thesis root](../../../../../README.md) > [simulation](../../../../README.md) > isaac > routes > 01_road > teach*


## Мета
Повторити teach-проїзд експ 59 pipeline на full road-roundtrip маршруті
(333 м, 167 WP), але з модифікованою моделлю IMU з експ 70 Run A:
synthetic-accel у motion-branch отримує Phidgets-1042 шум + bias
(раніше шум додавався лише коли робот стояв).

## Контекст
- Exp 59 - Pareto-оптимальна версія T&R pipeline (composite score #1 за трьома
  вагоми схемами). Використовується як baseline для дипломної роботи.
- Exp 65 - teach-запис на 4 маршрутах (road, south, north, other). Road teach
  завершився на 100% route, але IMU accel motion-branch був БЕЗ шуму -
  виявлено в аудиті exp 70.
- Exp 70 Run A підтвердив, що accel motion-branch noise injection **не погіршує**
  drift ORB-SLAM3 VIO (0.74 m -> 0.69 m). Тепер збираємо teach-артефакти з
  фізично правильною моделлю IMU.

## Конфігурація
- `run_husky_forest.py` = копія з `70_imu_fidelity/run_A_accel_noise/scripts/`
  (додає `accel_body += N(0, σ_phidgets) + bias` у motion-branch, рядки +-817-824).
- Pipeline такий самий як exp 65 teach: Isaac + VIO (RGB-D-Inertial, vio_th160)
  + tf_wall_clock_relay --use-gt + drift_monitor (gate 2 m) + teach_run_depth_mapper
  + visual_landmark_recorder.
- Маршрут: `route_memory/road/anchors.json` (167 WP, +-333 м, roundtrip).
- Drift-gate: 2 m abort, retry if triggered

## Артефакти для подальших кроків
- `teach/road/vio_pose_dense.csv` - 10 Hz VIO+GT трек (Procrustes-ready)
- `teach/road/traj_gt.csv` - dense GT trajectory
- `teach/road/teach_map*` - depth costmap (для repeat)
- `teach/road/landmarks.pkl` - visual anchors (для repeat матчингу)
- `teach/road/isaac.log`, `vio.log`, `tf_slam.log`, `drift_monitor.log`
- `teach/road/run_info.txt` - REC bag path

## Запуск
```
bash scripts/run_teach_single.sh road
```
Exit codes: 0 = success, 2 = drift abort (retry), 1 = other error.

## Результати

ROUTE COMPLETE - 4192 frames, full 333 m roundtrip, жодного drift abort.

| метрика | значення |
|---|---|
| gt_path | **321.40 m** |
| vio_path | 321.70 m |
| vio_scale | **1.0009** (0.09 % scale error) |
| drift_max | **0.69 m** |
| drift_mean | 0.45 m |
| GT start -> GT end | (-79.99, -1.40) -> (-91.88, -4.95) |
| landmarks.pkl | 120 KB |
| teach_map.pgm | 452 KB |
| traj_gt.csv | 126 KB |

Порівняння drift-vs-gt_path з експ 70:
| run | pipeline | drift_max | drift_mean |
|---|---|---|---|
| exp 70 baseline | master IMU, no teach-side | 0.74 | 0.49 |
| exp 70 Run A | +accel noise, no teach-side | 0.69 | 0.42 |
| **exp 66 teach** | +accel noise, **full teach pipeline** (depth+landmarks) | **0.69** | 0.45 |

**Висновок:** додавання teach-side outputs (depth_mapper, landmark_recorder)
не впливає на VIO drift - exp 66 ≈ exp 70 Run A у межах Isaac run-to-run шуму.
Teach-артефакти зібрано з фізично правильною моделлю IMU - готові для repeat pass.

Плоти: [teach/road/vio_vs_gt.png](teach/road/vio_vs_gt.png), [teach/road/drift_compare_with_exp70.png](teach/road/drift_compare_with_exp70.png).

## Структура landmarks.pkl
51 anchors over full 333 m roundtrip, з них **40 outbound + 11 return**
(turnaround @ anchor idx 39, x = +70.5 m). Кожен запис:
```python
{
  'pose':           [x, y, z, qx, qy, qz, qw],  # camera pose at capture
  'descriptors':    uint8 ndarray (N_kpts, 32),  # ORB descriptors
  'keypoints_2d':   float32 ndarray (N_kpts, 2),
  'keypoints_3d_cam': float32 ndarray (N_kpts, 3),
  'ts': float, 'n_features': int,
}
```
Pkl root: `intrinsics`, `base_to_cam_translation`, `base_to_cam_rot`, `landmarks`.   

## Відоме обмеження - anchor misfire на roundtrip маршруті
Під час exp 68 (repeat + obstacles) виявлено, що `visual_landmark_matcher`
може прийняти помилковий anchor на detour-heavy ділянці: у зоні tent+BARRIER 2
(dist 87–128 m) SLAM-nav tf стрибнув на +-14 m (nav=(15.9,−16.4) vs GT=(20.4,−3.0)).
Ймовірна причина - roundtrip візуальна амбівалентність: матчер у відхилному   
ракурсі збігся з descriptors від return-пасу (той самий фізичний x, інший yaw)
і PnP дав шибовану позу. Після turnaround self-correct через return-leg anchors.

Мітігація (exp 69): розділити landmarks.pkl на outbound/return підсети,
матчер використовує тільки outbound до turnaround, return після. Якщо це
знімає spike - зробити стандартом для всіх roundtrip teach-артефактів.

## Наступні кроки
- Exp 67+: Nav2 + ORB-SLAM3 repeat pass на зібраних teach-артефактах (x done)
- Exp 68: +obstacles (x 96% REACH, 14 m spike у detour зоні)
- Exp 69: split landmarks outbound/return (testing mitigation)
