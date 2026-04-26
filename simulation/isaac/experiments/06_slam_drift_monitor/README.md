# exp 06: SLAM + drift monitor v1 (return-to-path)

Detect when robot drifts too far from the planned trajectory and force a return to path maneuver. Idea: if SLAM thinks robot is on the road but mapped trajectory says it's far away, the SLAM is wrong -> emit a corrective goal.

## результат

**123 m, 67 % route, bypassed cones.** Better than exp 02-05.

But: **drift monitor oscillates at the cone area.**
