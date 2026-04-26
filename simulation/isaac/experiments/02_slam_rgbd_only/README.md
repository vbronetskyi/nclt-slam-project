# exp 02: Nav2 with RGBD-only ORB-SLAM3 (mapping mode)

Replace GT localization (exp 01) with real visual SLAM. ORB-SLAM3 RGBD mode runs in **mapping mode** (no atlas), builds a map from scratch during navigation. The relay reads `/tmp/slam_pose.txt` and computes a map->odom correction.

## результат

| wz_max | distance | route % | obstacles | Y drift | lost frames |
|---|---|---|---|---|---|
| 0.8 rad/s | 137 m | 75 % | 2/4 | 10 m | 274/970 = 28 % |
| 0.4 rad/s | 104 m | 57 % | 2/4 | 5.8 m | 7/2685 = 0.9 % |

