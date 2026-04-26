# exp 13: Regulated Pure Pursuit + larger inflation - best SLAM result (145 m)

Replace DWB with **Regulated Pure Pursuit (RPP)** controller. RPP has built-in features that DWB doesn't:
- Auto-slowdown near obstacles via `use_cost_regulated_linear_velocity_scaling`
- Auto-slowdown on tight turns via `use_regulated_linear_velocity_scaling`
- Smooth carrot-following without velocity-space sampling

Also: increase costmap inflation from 0.45 m -> **1.5 m globally, 1.0 m locally**. This gives the planner more room around obstacles, reducing the chance of robot driving too...

## результат

- best SLAM-based navigation result

**145 m, 87 % route, max X = 50 m. All 4 obstacle groups bypassed!**

| Metric | exp 13 (RPP) | exp 09 (DWB) | exp 01 (GT baseline) |
|---|---|---|---|
| Distance | **145 m** | 141 m | 182 m |
| Route % | **87 %** | 85 % | 100 % |
| Obstacles | **4/4** | 3/4 | 4/4 |
| Avg speed | 0.63 m/s | 0.72 m/s | 0.62 m/s |
