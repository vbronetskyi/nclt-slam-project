# exp 37: A* Local Planner on Obstacle Grid

Replace reactive gap navigation with A* path planning on the ego-centric
obstacle grid from exp 36. Instead of converting grid -> angular profile -> gap
selection, plan a short path directly from robot to lookahead anchor on the
100*100 costmap.

## результат

| Metric | Value |
|---|---|
| Mode | PLAN throughout |
| Path length | 7-10m (consistant) |
| Speed | **0.15 m/s** (4* slower than gap nav 0.62 m/s) |
| Grid cells | 48-72 (0.5-0.7%) |
| Impassable cells | 100-140 (roadside tree inflation) |
