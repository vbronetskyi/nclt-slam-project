#!/bin/bash
# Exp 55 REPEAT run - v53 base (proactive projection + proximity limiter)
#                    + v55 visual landmark matcher + tf_relay with
#                      /anchor_correction subscriber
set -eu

E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
E59=/workspace/simulation/isaac/routes/03_south/repeat
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

TEACH=$E59/teach
OUT=${REPEAT_OUT_DIR:-$E59/results/repeat_run}
mkdir -p $OUT/snapshots $OUT/plans

TEACH_MAP=$TEACH/south_teach_map.yaml
LANDMARKS=$TEACH/south_landmarks.pkl
[ -f "$TEACH_MAP" ]  || { echo "ERROR: missing teach map";         exit 1; }
[ -f "$LANDMARKS" ]  || { echo "ERROR: missing south_landmarks.pkl - run teach first"; exit 1; }

python3 - <<PY
import re
for p, pat in [
    ('$E59/config/nav2_planner_only.yaml', r'yaml_filename:.*'),
    ('$E59/config/nav2_launch_hybrid.py',  r'(map_yaml|config)\s*=\s*"[^"]*"'),
]:
    txt = open(p).read()
    if 'map_yaml' in pat:
        txt = re.sub(r'map_yaml = "[^"]*"', f'map_yaml = "$TEACH_MAP"', txt, count=1)
        txt = re.sub(r'config = "[^"]*"', f'config = "$E59/config/nav2_planner_only.yaml"', txt, count=1)
    else:
        txt = re.sub(pat, f'yaml_filename: "$TEACH_MAP"', txt, count=1)
    open(p, 'w').write(txt)
    print(f'Patched {p}')
PY

python3 $E52/scripts/patch_obstacles_exp52.py

# v59: no offline sanitization - goal sender does look-ahead skip + detour
# on live costmap at send time.  Uses original trajectory directly.
cp $E59/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
python3 -c "
import json
r = json.load(open('/tmp/slam_routes.json'))
r['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(r, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(r[\"south\"])} WPs')
"

pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall_clock|planner_server|map_server|pure_pursuit_path|send_goals_hybrid|lifecycle_manager|costmap_snapshotter|turnaround_supervisor|plan_logger|visual_landmark|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/exp55*.sh

echo "=== PHASE 1: Isaac + VIO warmup ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --obstacles --duration 1500 \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E59/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/exp55r_tf_gt.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp55r_tf_gt.sh
nohup bash /tmp/exp55r_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT warmup): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== PHASE 2: v55 tf_relay + anchor matcher + Nav2 + PP ==="
touch /tmp/isaac_clear_route.txt
echo "Pure pursuit DISABLED - waiting 5s for robot to settle"
sleep 5

kill $TF 2>/dev/null; sleep 2

cat > /tmp/exp55r_tf_slam.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E59/scripts/tf_wall_clock_relay_v55.py --slam-encoder
EOF
chmod +x /tmp/exp55r_tf_slam.sh
nohup bash /tmp/exp55r_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(SLAM v55): $TF"
sleep 3

cat > /tmp/exp55r_matcher.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E59/scripts/visual_landmark_matcher.py \
    --landmarks $LANDMARKS --out-csv $OUT/anchor_matches.csv
EOF
chmod +x /tmp/exp55r_matcher.sh
nohup bash /tmp/exp55r_matcher.sh > $OUT/landmark_matcher.log 2>&1 &
MATCHER=$!; disown $MATCHER; echo "LandmarkMatcher: $MATCHER"

cat > /tmp/exp55r_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $E59/config/nav2_launch_hybrid.py
EOF
chmod +x /tmp/exp55r_nav2.sh
nohup bash /tmp/exp55r_nav2.sh > $OUT/nav2.log 2>&1 &
NAV2=$!; disown $NAV2; echo "Nav2: $NAV2"

cat > /tmp/exp55r_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E59/scripts/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/exp55r_pp.sh
nohup bash /tmp/exp55r_pp.sh > $OUT/pp_follower.log 2>&1 &
PP=$!; disown $PP; echo "PP: $PP"

cat > /tmp/exp55r_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E59/scripts/turnaround_supervisor.py --turnaround-x 60.0 --past-margin 2.0
EOF
chmod +x /tmp/exp55r_sup.sh
nohup bash /tmp/exp55r_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP; echo "Supervisor: $SUP"

# CostmapSnapshotter disabled (disk)

cat > /tmp/exp55r_plan.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E59/scripts/plan_logger.py --out-dir $OUT/plans
EOF
chmod +x /tmp/exp55r_plan.sh
nohup bash /tmp/exp55r_plan.sh > $OUT/plan_logger.log 2>&1 &
PLN=$!; disown $PLN; echo "PlanLogger: $PLN"

for i in $(seq 1 60); do
    grep -q "Managed nodes are active\|planner_server.*active" $OUT/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready"

echo "=== PHASE 3: Send goals ==="
sleep 3
cat > /tmp/exp55r_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E59/scripts/send_goals_hybrid.py --trajectory /workspace/simulation/isaac/experiments/48_vio_roundtrip__/logs/exp48_vio_traj.csv --spacing 4.0 --goal-timeout 300 --tolerance 3.0
EOF
chmod +x /tmp/exp55r_goals.sh
nohup bash /tmp/exp55r_goals.sh > $OUT/goals.log 2>&1 &
GOALS=$!; disown $GOALS; echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Matcher=$MATCHER Nav2=$NAV2 PP=$PP Sup=$SUP CMS=$CMS Plan=$PLN Goals=$GOALS"
echo "OUT: $OUT"
echo ""
echo "Monitor:"
echo "  tail -f $OUT/tf_slam.log       # regime + anchor_age + err"
echo "  tail -f $OUT/anchor_matches.csv # every matcher attempt"
