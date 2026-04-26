# Exp 67 - Road repeat with accel-noise IMU

Запустити повний exp 59 repeat pipeline на road-маршруті,
використовуючи teach-артефакти з exp 66 (teach_map + landmarks) і   
synthetic-IMU з accel-шумом у motion-branch (exp 70 Run A). Це
повторює Pareto-оптимальну конфігурацію exp 59 (WP look-ahead skip,
detour, strict Nav2 clearance, SLAM-frame REACH check), але на road
маршруті та з фізично правильною IMU моделлю.

## результат

Pipeline відпрацював повністю. Усі 80 WP досягнуто.

| Метрика | Значення |
|---|---|
| **REACHED** | **80 / 80 (100 %)** |
| SKIP | 0 |
| DETOUR triggered | 3 (tent * 2, cone * 1) |
| Projections (costmap) | 2 |
| Duration | 830 s (≈ 14 min) |
| gt_path | 362.6 m |
