# exp 68 - Road repeat WITH obstacles + accel-noise IMU

Exp 59 Pareto-оптимальний pipeline на road маршруті з фізично коректною IMU
(accel noise у motion branch, exp 70 Run A), тепер **з повним набором
obstacles** (17 cones + 1 tent) на шляху outbound і їх автоматичним
видаленням після turnaround - відтворити класичний T&R detour-heavy   
сценарій exp 59 на road route.

## результат

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
