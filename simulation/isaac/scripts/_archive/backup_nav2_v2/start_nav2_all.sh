#!/bin/bash
# two-phase Nav2 navigation experiment
# phase 1: outbound with obstacles (SLAM or GT localization)
# phase 2: return without obstacles
#
# usage:
#   bash start_nav2_all.sh [--use-gt] [--route road|north|south]

set -e
SCRIPTS=/workspace/simulation/isaac/scripts
CONFIG=/workspace/simulation/isaac/config

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

ROUTE="${2:-road}"
USE_GT=""
for arg in "$@"; do
    case $arg in
        --use-gt) USE_GT="--use-gt" ;;
        --route=*) ROUTE="${arg#*=}" ;;
        road|north|south) ROUTE="$arg" ;;
    esac
done

echo "=== nav2 two-phase experiment ==="
echo "  route: $ROUTE"
echo "  localization: $([ -n "$USE_GT" ] && echo 'GT' || echo 'SLAM')"
echo ""

# clean up previous phase state
rm -f /tmp/nav2_phase.json

# 1. Isaac Sim
echo "[1/4] Isaac Sim..."
cd $SCRIPTS
/opt/isaac-sim-6.0.0/python.sh run_husky_nav2.py --route $ROUTE --duration 900 > /tmp/nav2_isaac.log 2>&1 &
ISAAC_PID=$!
echo "  PID: $ISAAC_PID"

echo "  waiting for scene load..."
for i in $(seq 1 180); do
    if grep -q "RUNNING" /tmp/nav2_isaac.log 2>/dev/null; then
        echo "  ready (${i}s)"
        break
    fi
    if ! kill -0 $ISAAC_PID 2>/dev/null; then
        echo "  FAILED - Isaac Sim exited"
        tail -10 /tmp/nav2_isaac.log
        exit 1
    fi
    sleep 1
done

# 2. TF: odom->world (identity) + SLAM TF publisher (map->odom)
echo "[2/4] TF publishers..."
source /opt/ros/jazzy/setup.bash

# TF relay: re-stamps Isaac Sim TF with wall clock + publishes map->odom + odom->world
# This solves sim_time vs wall_clock mismatch. Nav2 runs use_sim_time=false.
python3 $SCRIPTS/tf_wall_clock_relay.py $USE_GT > /tmp/nav2_tf.log 2>&1 &
TF_PID=$!
echo "  TF PID: $TF_PID"
sleep 2

# 3. Nav2
echo "[3/4] Nav2..."
ros2 launch $CONFIG/nav2_husky_launch.py use_sim_time:=false > /tmp/nav2_nav.log 2>&1 &
NAV2_PID=$!
echo "  PID: $NAV2_PID"

echo "  waiting for bringup..."
for i in $(seq 1 90); do
    if grep -q "Managed nodes are active" /tmp/nav2_nav.log 2>/dev/null; then
        echo "  active (${i}s)"
        break
    fi
    if grep -q "Failed to bring up" /tmp/nav2_nav.log 2>/dev/null; then
        echo "  FAILED!"
        tail -5 /tmp/nav2_nav.log
        kill $ISAAC_PID 2>/dev/null
        exit 1
    fi
    sleep 1
done

# 4. Two-phase goal sender
echo "[4/4] Goal sender (two-phase)..."
sleep 2
python3 $SCRIPTS/send_nav2_goal.py --route $ROUTE > /tmp/nav2_goal.log 2>&1 &
GOAL_PID=$!
echo "  PID: $GOAL_PID"

sleep 3
echo ""
echo "=== ALL RUNNING ==="
echo "  Isaac Sim: $ISAAC_PID  (log: /tmp/nav2_isaac.log)"
echo "  SLAM TF:   $TF_PID    (log: /tmp/nav2_tf.log)"
echo "  Nav2:      $NAV2_PID  (log: /tmp/nav2_nav.log)"
echo "  Goals:     $GOAL_PID  (log: /tmp/nav2_goal.log)"
echo ""
echo "  monitor goals: tail -f /tmp/nav2_goal.log"
echo "  monitor isaac: tail -f /tmp/nav2_isaac.log"
echo ""

# monitor loop
while true; do
    sleep 10
    if ! kill -0 $ISAAC_PID 2>/dev/null; then
        echo "Isaac Sim exited"
        break
    fi
    # show latest status
    POS=$(grep "  t=" /tmp/nav2_isaac.log 2>/dev/null | tail -1)
    PHASE=$(python3 -c "import json; print(json.load(open('/tmp/nav2_phase.json'))['phase'])" 2>/dev/null || echo "?")
    GOAL=$(grep -E "\[outbound\]|\[return\]|COMPLETE|PHASE" /tmp/nav2_goal.log 2>/dev/null | tail -1)
    echo "  phase=$PHASE | $POS | $GOAL"

    # check if done
    if [ "$PHASE" = "done" ]; then
        echo ""
        echo "=== NAVIGATION COMPLETE ==="
        sleep 5
        break
    fi
done

# save logs
RESULTS=/workspace/simulation/isaac/results/navigation
cp /tmp/nav2_isaac.log $RESULTS/nav2_isaac_${ROUTE}.log 2>/dev/null || true
cp /tmp/nav2_goal.log $RESULTS/nav2_goal_${ROUTE}.log 2>/dev/null || true
cp /tmp/nav2_tf.log $RESULTS/nav2_tf_${ROUTE}.log 2>/dev/null || true
echo "logs saved to $RESULTS/"

kill $GOAL_PID $NAV2_PID $TF_PID $ISAAC_PID 2>/dev/null || true
wait 2>/dev/null
echo "done"
