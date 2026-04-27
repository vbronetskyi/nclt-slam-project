#!/bin/bash
# Orchestrator: run one-or-more planned repeat routes (04..09) sequentially.
#
# Per route, same hygiene as run_all_teach.sh:
#   1. hard-kill leftover sim processes (Isaac, VIO, tf_relay, matcher,
#      planner_server, pure_pursuit, supervisor, goal_sender, plan_logger,
#      costmap_snapshotter)
#   2. wipe /dev/shm/fastrtps_* and /tmp flag/pose/obstacle files
#   3. run the route's run_repeat.sh under a wall-clock timeout
#   4. capture exit code + duration -> /tmp/run_all_repeat_summary.txt
# SIGINT/TERM: trap re-runs cleanup before exiting
#
# Usage:
#ROS_DOMAIN_ID=85 bash run_all_repeat.sh                 # all 04..09
#   bash run_all_repeat.sh 05_ne_sw                         # single route
#   OUTER_TIMEOUT_S=9000 bash run_all_repeat.sh 07_se_sw
set -u

ROUTES_DEFAULT=(04_nw_se 05_ne_sw 06_nw_ne 07_se_sw 08_nw_sw 09_se_ne)
if [ "$#" -gt 0 ]; then
    ROUTES=( "$@" )
else
    ROUTES=( "${ROUTES_DEFAULT[@]}" )
fi

OUTER_TIMEOUT_S=${OUTER_TIMEOUT_S:-9000}   # 2.5 h per route (repeat is slower than teach)
COOLDOWN_S=${COOLDOWN_S:-15}
KILL_PATTERN='run_husky_forest|rgbd_inertial|tf_wall_clock|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|turnaround_supervisor|planner_server|map_server|lifecycle_manager|plan_logger|costmap_snapshotter'

SUMMARY_FILE=/tmp/run_all_repeat_summary.txt
: > "$SUMMARY_FILE"

kill_all_sim() {
    echo "  [cleanup] pkill -9 repeat pipeline procs"
    pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
    sleep 3
    for attempt in 1 2 3 4 5; do
        REM=$(pgrep -fa "$KILL_PATTERN" 2>/dev/null | wc -l)
        [ "$REM" -eq 0 ] && break
        echo "  [cleanup] attempt $attempt: $REM still alive, retrying"
        pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
        sleep 3
    done
    REM=$(pgrep -fa "$KILL_PATTERN" 2>/dev/null | wc -l)
    [ "$REM" -gt 0 ] && { echo "  [cleanup] WARNING: $REM procs still alive"; pgrep -fa "$KILL_PATTERN" >&2 || true; }
    rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
    rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt 2>/dev/null
    rm -f /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_trajectory.csv 2>/dev/null
    rm -f /tmp/isaac_remove_obstacles.txt /tmp/isaac_clear_route.txt 2>/dev/null
    rm -f /tmp/teach_drift_abort.txt /tmp/teach_drift_status.txt 2>/dev/null
    rm -f /tmp/anchor_correction.csv 2>/dev/null
    echo "  [cleanup] done  (remaining: $REM)"
}

RUNNER_PID=""
RUNNER_PGID=""
on_interrupt() {
    echo ""
    echo "!!! repeat orchestrator received signal - cleanup"
    if [ -n "$RUNNER_PID" ] && kill -0 "$RUNNER_PID" 2>/dev/null; then
        kill -TERM -- "-$RUNNER_PGID" 2>/dev/null || true
        sleep 2
        kill -KILL -- "-$RUNNER_PGID" 2>/dev/null || true
    fi
    kill_all_sim
    echo "orchestrator interrupted" | tee -a "$SUMMARY_FILE"
    exit 130
}
trap on_interrupt INT TERM

echo ""
echo "RUN_ALL_REPEAT  started $(date +'%F %T')"
echo "routes: ${ROUTES[*]}"
echo "timeout/route: ${OUTER_TIMEOUT_S}s   cooldown: ${COOLDOWN_S}s"
echo ""

for R in "${ROUTES[@]}"; do
    BASE=/workspace/simulation/isaac/routes/$R/repeat
    SCRIPT=$BASE/scripts/run_repeat.sh
    if [ ! -x "$SCRIPT" ]; then
        echo "!!! $SCRIPT missing - skipping $R"
        echo "$R  SKIP  (missing)" >> "$SUMMARY_FILE"
        continue
    fi
    echo ""
    echo ""
    echo "=== REPEAT $R  start $(date +'%T') ==="
    echo ""

    kill_all_sim
    sleep "$COOLDOWN_S"

    OUT=$BASE/results/repeat_run
    mkdir -p "$OUT"
    LOG=$OUT/_orchestrator.log
    START=$(date +%s)
    setsid bash -c "timeout ${OUTER_TIMEOUT_S}s bash '$SCRIPT' 2>&1 | tee '$LOG'" &
    RUNNER_PID=$!
    RUNNER_PGID=$(ps -o pgid= -p "$RUNNER_PID" | tr -d ' ')
    set +e
    wait "$RUNNER_PID"
    CODE=$?
    set -e
    RUNNER_PID=""; RUNNER_PGID=""
    DUR=$(( $(date +%s) - START ))

    case "$CODE" in
        0)    STATUS="OK" ;;
        124)  STATUS="TIMEOUT" ;;
        130)  STATUS="INTERRUPTED" ;;
        *)    STATUS="FAIL(exit=$CODE)" ;;
    esac
    echo ""
    echo "=== REPEAT $R  $STATUS  ${DUR}s  at $(date +'%T') ==="
    printf "%-22s  %-16s  %6ds  exit=%-3s  %s\n" "$R" "$STATUS" "$DUR" "$CODE" "$LOG" >> "$SUMMARY_FILE"
done

kill_all_sim
echo ""
echo ""
echo "ALL REPEATS DONE  $(date +'%F %T')"
echo ""
cat "$SUMMARY_FILE"
