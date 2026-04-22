#!/bin/bash
# Gap Navigator + teach waypoints baseline on route 09_se_ne.
#
# Pipeline:
#   Isaac + obstacles
#   ORB-SLAM3 VIO  (same as ours)
#   tf_wall_clock_relay_v55 --slam-encoder  (same as ours - our tf)
#   visual_landmark_matcher                  (same as ours - anchor correction)
#   NO Nav2 stack.
#   turnaround_supervisor                    (same as ours - FIRE)
#   gap_nav_teach_client.py:
#       sub /camera/depth/image_rect_raw
#       sub tf (map->base_link)
#       read teach_outputs/vio_pose_dense.csv, subsample at 4 m
#       for each WP: compute heading error to WP, call GapNavigator
#       compute_cmd_vel(depth, heading_err) -> /cmd_vel
#       when d < 3 m -> next WP; timeout 180 s per WP (360 s for final 5)
set -eu

ROUTE=09_se_ne
E75=/workspace/simulation/isaac/experiments/75_gap_navigator_teach_wp
E72=/workspace/simulation/isaac/routes/$ROUTE/repeat
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-88}

TEACH=/root/isaac_tr_datasets/$ROUTE/teach/teach_outputs
OUT=${GAP_OUT_DIR:-$E75/results/run_09}
mkdir -p $OUT

LANDMARKS=$TEACH/landmarks.pkl
TRAJ=$TEACH/vio_pose_dense.csv
[ -f "$LANDMARKS" ] || { echo "ERROR: missing $LANDMARKS"; exit 1; }
[ -f "$TRAJ" ]      || { echo "ERROR: missing $TRAJ"; exit 1; }

pkill -9 -f "python.*run_husky_forest|rgbd_inertial|python.*tf_wall_clock|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|python.*pure_pursuit_path|python.*send_goals|python.*nav_through|python.*waypoint_follower_client|python.*gap_nav_teach|lifecycle_manager|python.*costmap_snapshotter|python.*turnaround_supervisor|python.*plan_logger|python.*visual_landmark|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/r75_*.sh

echo "=== PHASE 1: Isaac + VIO warmup (GT tf) ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:${PYTHONPATH:-}

python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py

setsid /opt/isaac-sim-6.0.0/python.sh $E72/scripts/run_husky_forest.py \
    --synthetic-imu --route $ROUTE --obstacles --duration 2400 \
    --spawn-x 65.00 --spawn-y -35.00 --spawn-yaw 1.0496 \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E72/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/r75_tf_gt.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/r75_tf_gt.sh
nohup bash /tmp/r75_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT warmup): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== PHASE 2: tf_slam v55 + matcher + supervisor (NO Nav2) ==="
touch /tmp/isaac_clear_route.txt
sleep 5
kill $TF 2>/dev/null || true; sleep 2

cat > /tmp/r75_tf_slam.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E72/scripts/tf_wall_clock_relay_v55.py --slam-encoder
EOF
chmod +x /tmp/r75_tf_slam.sh
nohup bash /tmp/r75_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(SLAM v55): $TF"
sleep 3

cat > /tmp/r75_matcher.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E72/scripts/visual_landmark_matcher.py \
    --landmarks $LANDMARKS --out-csv $OUT/anchor_matches.csv
EOF
chmod +x /tmp/r75_matcher.sh
nohup bash /tmp/r75_matcher.sh > $OUT/landmark_matcher.log 2>&1 &
MATCHER=$!; disown $MATCHER; echo "LandmarkMatcher: $MATCHER"

cat > /tmp/r75_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E72/scripts/turnaround_supervisor.py --final-x 65.0 --final-y 35.0 --near-radius 10.0
EOF
chmod +x /tmp/r75_sup.sh
nohup bash /tmp/r75_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP; echo "Supervisor: $SUP"

sleep 5

echo "=== PHASE 3: Gap Navigator client (teach WP follower) ==="
cat > /tmp/r75_client.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E75/scripts/gap_nav_teach_client.py \
    --trajectory $TRAJ --spacing 4.0
EOF
chmod +x /tmp/r75_client.sh
nohup bash /tmp/r75_client.sh > $OUT/goals.log 2>&1 &
CLIENT=$!; disown $CLIENT; echo "Client: $CLIENT"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Matcher=$MATCHER Sup=$SUP Client=$CLIENT"
echo "OUT: $OUT"

# WAIT_LOOP_FOR_RESULT
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-2400}
START_TS=$(date +%s)
cleanup_all() {
    pkill -TERM -f "visual_landmark_matcher|gap_nav_teach_client" 2>/dev/null || true
    sleep 3
    pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall_clock|turnaround_supervisor|visual_landmark_matcher|gap_nav_teach_client|ros2 launch" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT
while true; do
    ELAPSED=$(( $(date +%s) - START_TS ))
    if grep -q "RESULT:" "$OUT/goals.log" 2>/dev/null; then
        grep "RESULT:" "$OUT/goals.log" | tail -2
        cp /tmp/isaac_trajectory.csv "$OUT/traj_gt.csv" 2>/dev/null || true
        cleanup_all; trap - EXIT; exit 0
    fi
    [ "$ELAPSED" -gt "$ROUTE_TIMEOUT_S" ] && { echo "TIMEOUT after ${ELAPSED}s"; cleanup_all; exit 1; }
    sleep 5
done
