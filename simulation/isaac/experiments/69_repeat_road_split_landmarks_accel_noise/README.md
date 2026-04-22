# exp 69 - Road repeat, split outbound/return landmarks + accel-noise IMU

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 69_repeat_road_split_landmarks_accel_noise*


## Мета
Усунути 14 m anchor-misfire spike, виявлений у exp 68.

## Знахідка exp 68 (root cause)
Під час outbound детору (dist 87–128 m, зона tent + BARRIER 2) SLAM-nav tf
стрибнув на ~14 m:

| dist | nav (SLAM belief) | GT (truth) | err |
|---|---|---|---|
| 95 m | (0.9, **−1.4**) | (0.7, **+3.5**) | 4.9 m |
| 112 m | (6.9, **−10.4**) | (9.3, −1.3) | 9.5 m |
| 126 m | (15.9, **−16.4**) | (20.4, **−3.0**) | **14.1 m** |

GT тримається на дорозі (y ∈ [−3, +4]), але nav «переконаний» що робот
на y = −16 - ~13 m південніше ніж реально. Після turnaround drift
самовідновлюється до ~0.5 m.

**Механізм:** road - roundtrip route, і teach-landmarks.pkl містить
**обидва напрями** (40 outbound + 11 return anchors, turnaround ідея idx 39).
Під час detour робот з'їжджає далеко на південь, FOV змінюється. Matcher
серед 51 anchor знаходить «найсхожіший» - ним стає return-leg anchor
(той самий x але робот дивиться у протилежний бік, yaw ≈ +180°). PnP
обчислює `T_world_current` з return-anchor-pose + relative transform, що
дає робота на **симетричному боці дороги**. `HEADING_TOL_DEG = 90°` не
спрацював як захист - ймовірно через близькість yaw-різниці до порогу.

## Рішення (Step 1 - без re-teach)
Розділити **вже наявну** `landmarks.pkl` з exp 66 на два підсети по turnaround:
- landmarks_outbound.pkl (40 anchors, idx 0–39, x = −74.8 … +70.5)
- **landmarks_return.pkl**   (11 anchors, idx 40–50, x = +70.5 … −62.1)

## Рішення (Step 1 - без re-teach)
Розділити вже наявну `landmarks.pkl` з exp 66 на два підсети по turnaround:
- **landmarks_outbound.pkl** (40 anchors, idx 0–39, x = −74.8 … +70.5)
- **landmarks_return.pkl**   (11 anchors, idx 40–50, x = +70.5 … −62.1)

Matcher починає з outbound-set. Коли supervisor спрацьовує на turnaround,
він пише **обидва** прапорці:
- `/tmp/isaac_remove_obstacles.txt` - Isaac прибирає конуси+tent (як у exp 68)
- `/tmp/matcher_swap_return.txt` - matcher ре-завантажує return-set (нове)

## Зміни у скриптах
- **`scripts/split_landmarks.py`** - новий: post-process оригінального
  `landmarks.pkl`, записує два підсети зі спільною структурою pkl.
- **`scripts/visual_landmark_matcher.py`**:
  - CLI: додано `--landmarks-return`, `--swap-flag`
  - `__init__`: зберігає `return_pkl`, `swap_flag`
  - новий метод `_maybe_swap_to_return()` викликається першим у `_tick`
    - коли swap-flag існує, перечитує landmarks + xy + heading з
    return-pkl. Accumulation counter скидається (починаємо свіжу return-колекцію).
- `scripts/turnaround_supervisor.py`: у `fire()` тепер пише обидва
  flag-файли замість одного.
- **`scripts/run_exp69_repeat.sh`**: preprocess-крок (`split_landmarks.py`)
  + matcher викликається з обома pkl.

## Очікувані результати
- **Drift spike** у зоні tent+BARRIER 2 **має зникнути** - matcher бачить
  тільки outbound-anchors, які геометрично узгоджуються з поточною позою.
- REACH rate ≥ 96 % (як exp 68) або краще.
- Drift_mean менший, drift_max у діапазоні 1.5–3 m замість 14 m.

## Конфігурація обʼєктів (без змін від exp 68)
| Barrier | x | y range | cones |
|---|---|---|---|
| BARRIER 1 | −50 | [−8.0, −3.0] | 6 |
| BARRIER 2 | +15 | [−1.0, +4.0] | 6 |
| BARRIER 3 | +45 | [−3.0, +1.0] | 5 |
| Tent | (−20, 0) | 2.0 × 1.8 m | 1 |

**Обʼєкти прибираються при x ≥ 70 (supervisor trigger).**

## Запуск
```
bash scripts/run_exp69_repeat.sh
```

## Наступні кроки
- Якщо spike зникає -> стандартизувати split у всіх repeat-оркестраторах для roundtrip teach
- Якщо spike лишається - root cause не ambiguity outbound↔return, а щось інше
  (наприклад, within-outbound visual similarity) -> тюнити HEADING_TOL_DEG
  або додавати translation-shift outlier rejection
