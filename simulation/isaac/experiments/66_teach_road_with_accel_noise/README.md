# exp 66 - Road teach with accel-noise IMU

Повторити teach-проїзд експ 59 pipeline на full road-roundtrip маршруті
(333 м, 167 WP), але з модифікованою моделлю IMU з експ 70 Run A:
synthetic-accel у motion-branch отримує Phidgets-1042 шум + bias
(раніше шум додавався лише коли робот стояв).

## результат

**ROUTE COMPLETE** - 4192 frames, full 333 m roundtrip, жодного drift abort.

| метрика | значення |
|---|---|
| gt_path | **321.40 m** |
| vio_path | 321.70 m |
| vio_scale | **1.0009** (0.09 % scale error) |
| drift_max | **0.69 m** |
| drift_mean | 0.45 m |
| GT start -> GT end | (-79.99, -1.40) -> (-91.88, -4.95) |
