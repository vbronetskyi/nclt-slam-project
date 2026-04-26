# Exp 35: Road Obstacle Avoidance

Evaluate the teach-and-repeat pipeline on the road route with physical collision
obstacles (traffic cones + camping tent). The road environment (6-8m wide, flat
surface) eliminates the forest-specific problems identified in exp 34
(render/collision desync, dense vegetation blocking depth). Here, obstacles have
both render and collision meshes - depth sees them correctly.

**Research question**: Can depth-based gap navigation detect and bypass real
obstacles during route replay?

## результат

| Obstacle | X pos | Y positions | Route y | Distance from start |
|---|---|---|---|---|
| Cone group 1 (3) | -50 | -5.5, -4.5, -3.5 | -4.8 | 42m (25%) |
| Tent | -20 | 0.0 | 0.0 | 72m (43%) |
| Cone group 2 (3) | 15 | -3.0, -2.0, -1.0 | -2.0 | 108m (65%) |
| Cone group 3 (3) | 45 | -2.0, -1.0, 0.0 | -1.2 | 139m (83%) |

## проблеми

1. Cone gap too narrow: 0.4m physical gap vs 0.67m robot width
- Between-cone routing: Gap navigator selects gaps BETWEEN individual
   cones rather than AROUND the entire group
3. **Post-pass pinning**: Robot passes through gap but contacts cone on exit,
   gets pinned at y=-5.1 (between y=-5.5 and y=-4.5 cones)
