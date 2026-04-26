# exp 40: Teach-and-Repeat + Nav2

Single destination goal (70.4, -2.3) sent to Nav2. Nav2 plans on blank static
map, uses depth camera for obstacle detection, replans around obstacles.

## результат

| # | Issue | Fix |
|---|---|---|
| 1 | Camera API depth broken | OmniGraph ROS2CameraHelper |
| 2 | No odom TF frame | GT mode: map->odom->base_link |
| 3 | TF_OLD_DATA conflict | OmniGraph TF remapped to /tf_isaac |
| 4 | Static map loading old SLAM | Blank 900*200 map (180*40m) |
| 5 | Pose file stale | Bridge writes GT to isaac_pose.txt |

## проблеми

fixed in this experiment

| # | Issue | Fix |
|---|---|---|
| 1 | Camera API depth broken | OmniGraph ROS2CameraHelper |
| 2 | No odom TF frame | GT mode: map->odom->base_link |
