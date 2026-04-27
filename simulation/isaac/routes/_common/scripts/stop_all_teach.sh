#!/bin/bash
# Stop a run_all_teach.sh orchestrator and clean up every sim process.
# Safe to run whether or not one is currently active
KILL_PATTERN='run_husky_forest|rgbd_inertial|tf_wall_clock|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|turnaround_supervisor'

ORCH_PIDS=$(pgrep -f 'run_all_teach.sh' 2>/dev/null | grep -v $$ || true)
if [ -n "$ORCH_PIDS" ]; then
    echo "orchestrator pids: $ORCH_PIDS -> SIGTERM"
    kill -TERM $ORCH_PIDS 2>/dev/null || true
    sleep 4
    # finish off anything still alive
    STILL=$(pgrep -f 'run_all_teach.sh' 2>/dev/null | grep -v $$ || true)
    [ -n "$STILL" ] && { echo "still alive: $STILL -> SIGKILL"; kill -9 $STILL; }
else
    echo "no orchestrator running"
fi

echo "hard-kill sim processes matching: $KILL_PATTERN"
pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
sleep 2
for i in 1 2 3; do
    REM=$(pgrep -f "$KILL_PATTERN" 2>/dev/null | wc -l)
    [ "$REM" -eq 0 ] && break
    echo "  retry $i: $REM sim procs still alive"
    pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
    sleep 2
done

rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt 2>/dev/null
rm -f /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_trajectory.csv 2>/dev/null
rm -f /tmp/teach_drift_abort.txt /tmp/teach_drift_status.txt /tmp/vio_pose_dense.csv 2>/dev/null

REM=$(pgrep -f "$KILL_PATTERN" 2>/dev/null | wc -l)
echo "done  (sim procs remaining: $REM)"
[ "$REM" -eq 0 ] && exit 0 || exit 1
