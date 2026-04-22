# Experiment 00: First Autonomous Navigation

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > gazebo > experiments > 00_navigation_test*


## Setup
- Date: 2026-03-07
- Robot: Husky A200 sim (skid-steer, 50kg, 990x670mm)
- World: outdoor_terrain 50x50m (trees, rocks, bushes)
- SLAM: SLAM Toolbox (LiDAR-based, async mode)
- Planner: NavFn (A*), allow_unknown: true
- Controller: Regulated Pure Pursuit (RPP)
- Global costmap: 30x30m rolling window, 0.05m resolution

## Result: SUCCESS
- Map built: 20,473 free cells in 3 exploration loops (~90s)
- Auto-goal found: (0.85, -1.66), 3.42m from robot in verified free space
- Navigation: 2.70m to 0.44m in 7.2 seconds
- Final distance to goal: 1.00m (within tolerance)
- Status: 4 (SUCCEEDED)

## Key Fixes Applied
1. DWB -> RPP controller, DWB couldn't find valid trajectories for skid-steer
2. RPP collision_detection: false, was detecting phantom collisions, collision_monitor handles safety
3. inflation_radius: 0.55, matches inscribed radius of Husky footprint
4. cost_scaling_factor: 2.0, softer inflation falloff
5. global costmap: 30x30m rolling_window, prevents unbounded growth that killed planner_server
6. auto-goal in mapped free space, makes sure the goal is reachable

## Reproduction
```bash
cd /workspace/simulation
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=64
ros2 launch ugv_gazebo full_sim.launch.py headless:=true slam_type:=slam_toolbox &
sleep 55
python3 experiments/00_navigation_test/build_map_then_navigate.py
```
