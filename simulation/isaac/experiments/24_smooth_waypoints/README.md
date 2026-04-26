# exp 24: Smooth waypoints for forest VIO

After fixing IMU calibration in exp 23 (VIO road ATE 0.089m), test if **smoothing forest waypoints** (removing A* planner's artificial U-turns) enables VIO to work on forest routes.

## результат

| Route | Original wp | Smoothed wp | Length | Rotation |
|---|---|---|---|---|
| North | 235 | 118 (-50%) | 487m -> 442m | 2846° -> 1561° (-45%) |
| South | 201 | 99 (-51%) | 401m -> 369m | 1566° -> 1066° (-32%) |
