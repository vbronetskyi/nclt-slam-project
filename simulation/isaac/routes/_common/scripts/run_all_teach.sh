#!/bin/bash
# Orchestrator: run every planned teach route (04..09) sequentially.
#
# For each route:
#   1. hard-kill any leftover sim process (Isaac, VIO, tf_relay, recorders,
#      depth mapper, drift monitor, supervisor)  - wait until pgrep count = 0
#   2. wipe /dev/shm/fastrtps_* and /tmp pose/flag files so nothing from the
#      previous run leaks in
#   3. run that route's run_teach_with_retry.sh under an outer wall-clock
#      timeout (default 2 h per route - more than enough: each teach is
#      10–40 min, the retry wrapper allows up to 3× internal attempts)
#   4. capture exit code + duration, move on to the next
# On SIGINT/SIGTERM the trap re-runs cleanup before exiting so nothing is
# left orphaned
#
# Usage:
#   ROS_DOMAIN_ID=85 bash run_all_teach.sh
#   OUTER_TIMEOUT_S=7200 COOLDOWN_S=15 bash run_all_teach.sh 06_nw_ne 07_se_sw
set -u

ROUTES_DEFAULT=(04_nw_se 05_ne_sw 06_nw_ne 07_se_sw 08_nw_sw 09_se_ne)
if [ "$#" -gt 0 ]; then
    ROUTES=( "$@" )
else
    ROUTES=( "${ROUTES_DEFAULT[@]}" )
fi

OUTER_TIMEOUT_S=${OUTER_TIMEOUT_S:-8400}   # 2 h hard ceiling per route
COOLDOWN_S=${COOLDOWN_S:-10}               # wait after cleanup before next start
KILL_PATTERN='run_husky_forest|rgbd_inertial|tf_wall_clock|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|turnaround_supervisor'

SUMMARY_FILE=/tmp/run_all_teach_summary.txt
: > "$SUMMARY_FILE"

kill_all_sim() {
    echo "  [cleanup] pkill -9 $KILL_PATTERN"
    pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
    sleep 3
    for attempt in 1 2 3 4 5; do
        REM=$(pgrep -fa "$KILL_PATTERN" 2>/dev/null | wc -l)
        if [ "$REM" -eq 0 ]; then
            break
        fi
        echo "  [cleanup] attempt $attempt: $REM sim procs still alive, retrying..."
        pgrep -fa "$KILL_PATTERN" 2>/dev/null | head -5
        pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
        sleep 3
    done
    REM=$(pgrep -fa "$KILL_PATTERN" 2>/dev/null | wc -l)
    if [ "$REM" -gt 0 ]; then
        echo "  [cleanup] WARNING: $REM sim processes still alive after kill" >&2
        pgrep -fa "$KILL_PATTERN" >&2 || true
    fi
    rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
    rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt 2>/dev/null
    rm -f /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_trajectory.csv 2>/dev/null
    rm -f /tmp/isaac_remove_obstacles.txt /tmp/isaac_clear_route.txt 2>/dev/null
    rm -f /tmp/teach_drift_abort.txt /tmp/teach_drift_status.txt /tmp/vio_pose_dense.csv 2>/dev/null
    echo "  [cleanup] done  (sim procs: $REM)"
}

RUNNER_PID=""
RUNNER_PGID=""

on_interrupt() {
    echo ""
    echo "!!! orchestrator received signal - cleaning up and exiting"
    if [ -n "$RUNNER_PID" ] && kill -0 "$RUNNER_PID" 2>/dev/null; then
        echo "  [interrupt] killing runner pgid=-$RUNNER_PGID"
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
echo "RUN_ALL_TEACH  started $(date +'%F %T')"
echo "routes: ${ROUTES[*]}"
echo "timeout/route: ${OUTER_TIMEOUT_S}s   cooldown: ${COOLDOWN_S}s"
echo "summary: $SUMMARY_FILE"
echo ""

for R in "${ROUTES[@]}"; do
    BASE=/workspace/simulation/isaac/routes/$R/teach
    SCRIPT=$BASE/scripts/run_teach_with_retry.sh
    if [ ! -x "$SCRIPT" ]; then
        echo "!!! $SCRIPT missing or not executable - skipping $R"
        echo "$R  SKIP  (missing script)" >> "$SUMMARY_FILE"
        continue
    fi

    echo ""
    echo ""
    echo "=== ROUTE $R  start $(date +'%T') ==="
    echo ""

    kill_all_sim
    sleep "$COOLDOWN_S"

    LOG=$BASE/teach/$R/_orchestrator.log
    mkdir -p "$(dirname "$LOG")"
    START=$(date +%s)

    # Run the teach in its own process group (setsid) so we can kill the
    #entire subtree on signal. Stream stdout to the orchestrator stdout
    # and to $LOG via `tee`; use &+wait so the trap actually fires.
    setsid bash -c "timeout ${OUTER_TIMEOUT_S}s bash '$SCRIPT' '$R' 2>&1 | tee '$LOG'" &
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
        2)    STATUS="DRIFT_ABORT" ;;
        *)    STATUS="FAIL(exit=$CODE)" ;;
    esac
    echo ""
    echo "=== ROUTE $R  $STATUS  ${DUR}s  at $(date +'%T') ==="
    printf "%-22s  %-16s  %6ds  exit=%-3s  %s\n" "$R" "$STATUS" "$DUR" "$CODE" "$LOG" >> "$SUMMARY_FILE"
done

kill_all_sim
echo ""
echo ""
echo "ALL ROUTES DONE  $(date +'%F %T')"
echo ""
cat "$SUMMARY_FILE"
