# exp 59 - WP Look-Ahead Skip + Detour

## результат

| Area | Exp 58 | Exp 59 |
|---|---|---|
| WP safety handling | Offline `sanitize_route.py` shifts WP <=2 m | Runtime look-ahead skip + detour on 4–7 m ring |
| Known obstacle check | - | Hard-coded cone/tent positions (costmap-independent) |
| Min clearance from obstacle centre | +-2.0 m (shift) | >=1.6 m (robot edge >= 0.9 m from obstacle edge) |
| `planner.tolerance` | 1.0 m | 0.3 m |
| `robot_radius` | 0.5 m | 0.7 m |
