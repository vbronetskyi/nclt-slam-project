#!/bin/bash
# Exp 55 TEACH run - drives south roundtrip without obstacles, captures:
#   - depth occupancy map (teach_run_depth_mapper.py, same as exp 52)
#   - visual landmarks every 2m of VIO displacement
#     (visual_landmark_recorder.py)
# Output: teach/south_teach_map.{pgm,yaml}, teach/south_landmarks.pkl
set -eu

E55=/workspace/simulation/isaac/experiments/55_visual_teach_repeat
E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

TEACH=$E55/teach
OUT=${TEACH_OUT_DIR:-$TEACH}
mkdir -p $OUT

pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/exp55*.sh

echo "=== TEACH PHASE 1: Isaac (NO obstacles) + VIO warmup ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --duration 1500 \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E55/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

# tf_relay in GT mode for teach - this is the design: teach is the
# "known-good" run.  It records what would be VIO during repeat.  We
# still use GT for tf here to give the depth mapper clean poses.
cat > /tmp/exp55t_tf.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp55t_tf.sh
nohup bash /tmp/exp55t_tf.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== TEACH PHASE 2: depth-map + landmark recorders + auto-drive ==="

cat > /tmp/exp55t_depth.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/teach_run_depth_mapper.py \
    --out-prefix $TEACH/south_teach_map \
    --origin-x -110 --origin-y -50 --width-m 200 --height-m 60 --res 0.1
EOF
chmod +x /tmp/exp55t_depth.sh
nohup bash /tmp/exp55t_depth.sh > $OUT/teach_depth_mapper.log 2>&1 &
DEPTH=$!; disown $DEPTH; echo "DepthMapper: $DEPTH"

cat > /tmp/exp55t_lm.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E55/scripts/visual_landmark_recorder.py \
    --out $TEACH/south_landmarks.pkl --min-disp 2.0
EOF
chmod +x /tmp/exp55t_lm.sh
nohup bash /tmp/exp55t_lm.sh > $OUT/landmark_recorder.log 2>&1 &
LM=$!; disown $LM; echo "LandmarkRecorder: $LM"

echo ""
echo "Teach running. Isaac open-loop drives the route. Let run to completion (~15 min)."
echo "Pids: Isaac=$ISAAC VIO=$VIO TF=$TF Depth=$DEPTH LM=$LM"
echo "Monitor:"
echo "  tail -f $OUT/landmark_recorder.log"
echo "  tail -f $OUT/teach_depth_mapper.log"
