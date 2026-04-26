# exp 03: Nav2 with ORB-SLAM3 localization mode (pre-built atlas)

Improve over exp 02 (mapping mode, drift) by giving SLAM a pre-built reference map. ORB-SLAM3 localization mode loads an `.osa` atlas built offline from a clean mapping run on the same route.

## результат

**110 m, 60 % route, cones bypassed.** 28/1665 lost frames (1.7 %).

| Metric | Localization (exp 03) | Mapping (exp 02) |
|---|---|---|
| distance | 110 m (60 %) | 106 m (58 %) |
| Y drift | 3-5 m | 5-10 m |
| lost frames | 1.7 % | 0.1-28 % |
| stuck at | x=-23 (after tent) | x=-23 (after tent) |

Stuck at the same world location as exp 02 (x=-23, y=9) after the tent bypass.
