#!/bin/bash
# Shared IMU-fidelity test drive: 100 m out + 180° turnaround + 100 m back
# on the road, no obstacles, pure-pursuit-style autopilot in
# run_husky_forest.py. Used by baseline / run_A / run_B / run_C.
#
# Args:   
#   $1  out_dir          - results destination (mandatory)
#   $2  husky_script     - path to run_husky_forest.py variant to run
#                          (baseline uses master; run_A/B use modified copies)
#   $3  warmup_s         - seconds of stationary hold after VIO init (for run_B)
#                          default 0 (no extra warmup)
set -u
set -x

OUT=${1:?usage: $0 <out_dir> <husky_script> [warmup_s]}
HUSKY_SCRIPT=${2:?husky script}
WARMUP_S=${3:-0}
DOMAIN=${ROS_DOMAIN_ID:-85}
E70=/workspace/simulation/isaac/experiments/70_imu_fidelity
SLAM=/workspace/third_party/ORB_SLAM3

mkdir -p $OUT

echo "=== exp 70 test drive ==="
echo "  out:  $OUT"
echo "  husky_script: $HUSKY_SCRIPT"
echo "  warmup_s: $WARMUP_S"

pkill -9 -f "python.*run_husky_forest|rgbd_inertial|python.*tf_wall|python.*visual_landmark|python.*vio_drift_monitor|python.*teach_run_depth|python.*pure_pursuit|python.*send_goals" 2>/dev/null || true
sleep 4
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/teach_drift_abort.txt /tmp/teach_drift_status.txt

source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN

setsid /opt/isaac-sim-6.0.0/python.sh $HUSKY_SCRIPT \
    --synthetic-imu --route road_nav2 --duration 600 \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E70/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/exp70_tf.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp70_tf.sh
nohup bash /tmp/exp70_tf.sh > $OUT/tf.log 2>&1 &
TF=$!; disown $TF; echo "TF: $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

# Optional static warmup (run_B) - hold robot still for N sim seconds after VIO ready
if [ "$WARMUP_S" -gt 0 ]; then
    echo "static warmup $WARMUP_S s ..."
    #send a /tmp marker that run_husky_forest reads to freeze autopilot
    touch /tmp/isaac_hold_autopilot.txt
    sleep $WARMUP_S
    rm -f /tmp/isaac_hold_autopilot.txt
    echo "warmup done"
fi

# Start drift monitor
cat > /tmp/exp70_drift.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=$DOMAIN
exec python3 $E70/scripts/vio_drift_monitor.py \
    --out $OUT/vio_pose_dense.csv --max-drift-m 10.0 \
    --check-interval-s 10 --settling-s 30
EOF
chmod +x /tmp/exp70_drift.sh
nohup bash /tmp/exp70_drift.sh > $OUT/drift_monitor.log 2>&1 &
DRIFT=$!; disown $DRIFT; echo "DriftMonitor: $DRIFT"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Drift=$DRIFT"
echo ""

cleanup_all() {
    pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall|vio_drift_monitor" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT

# Wait for ROUTE COMPLETE. Use 1 h wall cap.
START_TS=$(date +%s)
while true; do
    NOW=$(date +%s); EL=$((NOW - START_TS))
    if grep -q "ROUTE COMPLETE" $OUT/isaac.log 2>/dev/null; then
        echo "=== ROUTE COMPLETE after ${EL}s wall ==="
        cp /tmp/isaac_trajectory.csv $OUT/traj_gt.csv 2>/dev/null || true
        [ -f /tmp/teach_drift_status.txt ] && echo "final: $(cat /tmp/teach_drift_status.txt)"
        cleanup_all
        trap - EXIT
        exit 0
    fi
    if [ "$EL" -gt 3600 ]; then
        echo "!!! wall timeout"; cleanup_all; exit 1
    fi
    sleep 5
done
