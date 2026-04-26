# exp 01: Nav2 with GT localization (baseline)

Establish a baseline for Nav2-based navigation in Isaac Sim using ground-truth (GT) localization. This is the upper bound - anything that uses real SLAM should be compared against this.

## результат

- **182 m, 100 % route, 4/4 obstacles bypassed**, 294 s
- Avg speed 0.62 m/s (0.8 on straights, slowdown at obstacles)
- Cone group 1 (x=-50): bypassed south at y=-6.9
- Tent (x=-20): backup + replan, +-90 s delay
- Cone groups 2 and 3: passed without stopping
- Return: 35/35 waypoints, clean road
