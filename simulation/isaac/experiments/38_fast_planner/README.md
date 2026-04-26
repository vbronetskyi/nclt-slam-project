# exp 38: Optimized A* Planner

Optimize A* planner from exp 37 to maintain higher speed. Exp 37 proved A*
plans correctly around cones but robot moved at 0.15 m/s (too slow to execute
the lateral detour). Target: >0.4 m/s, bypassing cone group 1.

## результат

| Optimization | Exp 37 | Exp 38 |
|---|---|---|
| Planner engine | GridPlanner (scipy) | FastGridPlanner (scipy + cache) |
| Replan frequency | Every 3 frames | Every 10 frames (cached) |
| A* max iterations | 8000 | 3000 + early termination |
| Planning time | 60-100ms | **3-9ms** |
| Speed controller | Binary (0.55/0.15) | 3-tier (0.6/0.4/0.2) |
