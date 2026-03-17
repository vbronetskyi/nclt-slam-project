# exp 01: Nav2 with GT localization (baseline)

## Goal

Establish a baseline for Nav2-based navigation in Isaac Sim using ground-truth (GT) localization. This is the upper bound - anything that uses real SLAM should be compared against this.

## setup

- Robot: Husky A200 with D435i RGB-D camera, PhysX dynamics, 200 Hz physics
- Localization: GT pose from Isaac Sim written to `/tmp/isaac_pose.txt` every 3rd frame (~20 Hz), republished as TF `world->base_link` by `tf_wall_clock_relay.py --use-gt`
- Nav2 stack: NavFn global planner, DWB local controller (replaced MPPI which was 0.06 m/s)
- Static map: SLAM-built occupancy grid from prior mapping run (`results/navigation/slam_map/map.yaml`)
- Obstacle layer: depth-camera point cloud in global costmap with `max_obstacle_height: 1.2` (passes cones, blocks tree trunks) and `obstacle_max_range: 4.0`
- Two phases: outbound with 4 obstacle groups (3 cone groups + tent), then return after `/tmp/nav2_phase.json` flips to `removing`

## key engineering work

| Problem | Cause | Fix |
|---|---|---|
| MPPI controller stuck at 0.06 m/s | MPPI sampling underperforms on skid-steer | Replaced with DWB (samples velocity space directly) |
| Isaac TF on `/tf` had `sim_time` stamps | OmniGraph PublishTF ignores topic remap | Removed all TF/odom from OmniGraph, relay publishes with wall clock |
| `depth_image_proc` couldn't sync depth+camera_info | sim_time mismatch in messages | Built `depth -> PointCloud2` directly in relay |
| Trees blocked the road in costmap | depth obstacle layer marked tree trunks | `max_obstacle_height: 1.2` (cones 1m pass, trunks 5m+ filtered) |
| velocity_smoother lifecycle failed | unmaintained/buggy in Nav2 jazzy | Removed; controller publishes directly to `/cmd_vel` |
| DDS zombie participants | killed processes leave shared memory | Unique `ROS_DOMAIN_ID` per run + clean `/dev/shm/fastrtps_*` |

## Result - best run

- **182 m, 100 % route, 4/4 obstacles bypassed**, 294 s
- Avg speed 0.62 m/s (0.8 on straights, slowdown at obstacles)
- Cone group 1 (x=-50): bypassed south at y=-6.9
- Tent (x=-20): backup + replan, ~90 s delay
- Cone groups 2 and 3: passed without stopping
- Return: 35/35 waypoints, clean road

## files

- `config/nav2_husky_params.yaml` - DWB params, depth obstacle layer, costmap settings
- `config/nav2_husky_launch.py` - Nav2 stack launch (no velocity_smoother)
- `config/nav2_bt_simple.xml` - BT with `backup(1.0)` + `wait(3s)`, no spin (bad for skid-steer), 8 retries, 1 Hz replan
- `scripts/run_husky_nav2.py` - Isaac Sim host script
- `scripts/tf_wall_clock_relay.py` - TF + odom + IMU relay
- `scripts/send_nav2_goal.py` - two-phase waypoint sender
- `results/trajectory_plot.png` - full route plot

## plot

`results/trajectory_plot.png` - robot trajectory on the road map with obstacles. Shows successful traversal of all 4 obstacle groups in both directions.

## why it matters

GT localization removes the SLAM/drift problem and lets us validate the rest of the navigation stack (Nav2 config, obstacle avoidance, BT, costmap layers). All later SLAM experiments use the same Nav2 setup, only the localization source changes.
