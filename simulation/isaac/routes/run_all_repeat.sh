#!/bin/bash
# Top-level entry for sequential route repeats (04..09).
# Defaults: background via nohup, stdout -> /workspace/simulation/isaac/routes/_common/runs/<ts>.log.
#
# Usage:
#   bash routes/run_all_repeat.sh               # all 6, background
#   bash routes/run_all_repeat.sh --fg          # foreground (blocks)
#   bash routes/run_all_repeat.sh 05 06         # subset (short name expansion)
#   bash routes/_common/scripts/stop_all_teach.sh   # stop (shared stop script also kills repeat)
set -eu

SCRIPT_DIR=/workspace/simulation/isaac/routes/_common/scripts
ORCH=$SCRIPT_DIR/run_all_repeat.sh

MODE=bg
ARGS=()
for a in "$@"; do
    if [ "$a" = "--fg" ]; then
        MODE=fg
    else
        case "$a" in
            04) ARGS+=(04_nw_se) ;;
            05) ARGS+=(05_ne_sw) ;;
            06) ARGS+=(06_nw_ne) ;;
            07) ARGS+=(07_se_sw) ;;
            08) ARGS+=(08_nw_sw) ;;
            09) ARGS+=(09_se_ne) ;;
            *)  ARGS+=("$a") ;;
        esac
    fi
done

TS=$(date +%Y%m%d_%H%M%S)
LOG=/workspace/simulation/isaac/routes/_common/runs/run_all_repeat_$TS.log
mkdir -p "$(dirname "$LOG")"

echo "=== run_all_repeat ==="
echo "  routes: ${ARGS[*]:-04..09 (all)}"
echo "  mode:   $MODE"
echo "  log:    $LOG"
echo "  stop:   bash $SCRIPT_DIR/stop_all_teach.sh"
echo "  summary: /tmp/run_all_repeat_summary.txt"
echo ""

if [ "$MODE" = "fg" ]; then
    exec bash "$ORCH" "${ARGS[@]}"
else
    nohup bash "$ORCH" "${ARGS[@]}" >"$LOG" 2>&1 &
    ORCH_PID=$!
    disown "$ORCH_PID" || true
    echo "orchestrator pid: $ORCH_PID  ->  $LOG"
    echo "$ORCH_PID" > /tmp/run_all_repeat.pid
    echo "$LOG"      > /tmp/run_all_repeat.log_path
fi
