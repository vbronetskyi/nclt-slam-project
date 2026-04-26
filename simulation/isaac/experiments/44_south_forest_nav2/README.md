# exp 44: South Forest Route with Nav2

Test proven Nav2 + teach-repeat architecture (exp 41-43) on the challenging
south forest route. Establish whether forest passages are feasible with
depth costmap obstacle avoidance.

## результат

Nav2 + teach-repeat fails on dense forest - both sparse and dense waypoints hit the same obstacle cluster at (-19, -20). Route ceiling +-85m of 196m regardless of Nav2 tuning.

| Scenario | Result | Distance covered |
|---|---|---|
| Road with obstacles (exp 41-43) | 95% (41/43) | 183m of 167m |
| **South forest, v1 (4m WPs, inflation 0.5m)** | **23/46 (50%)** | **+-85m** - stuck at (-19.1, -21.6) |
| **South forest, v2 (2m WPs, inflation 0.3m)** | **50/99 (51%)** | **+-71m** - stuck at (-19.4, -19.8) |

Both runs hit the **same oak+shrub cluster around x≈-19**. Dense waypoints
and lower...
