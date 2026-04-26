# exp 09: SLAM Frame Navigation - paradigm shift

Stop fighting SLAM drift in world frame. Instead, **navigate entirely in SLAM coordinate frame**: waypoints and robot pose are both in the same drifted frame, so drift cancels out.

## результат

- best SLAM result up to that point

**141 m, 85 % route, 3/4 obstacle groups bypassed!** Drift consistently 0.3-1.5 m on straights.

| Metric | exp 09 (SLAM frame) | exp 07 (best world frame) |
|---|---|---|
| Distance | **141 m** | 138 m |
| Route % | **85 %** | 76 % |
| Obstacles bypassed | 3/4 | 4/4 (but stuck after) |
| Lateral drift on straights | 0.3-1.5 m | 5-8 m |
