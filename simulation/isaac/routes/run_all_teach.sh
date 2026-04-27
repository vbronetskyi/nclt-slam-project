#!/bin/bash
# Top-level entry point: teach every planned route (04..09) in sequence.
#
# Defaults:
#   - runs in background via nohup, disowned from this shell
#   - stdout + stderr streamed to $LOG
#- SUMMARY_FILE shows per-route status (OK / TIMEOUT / DRIFT_ABORT / FAIL)
#
#Usage:
#   bash routes/run_all_teach.sh              # background, all 6 routes
#   bash routes/run_all_teach.sh --fg         # foreground (blocks terminal)
#   bash routes/run_all_teach.sh 06 07        # only these routes
#bash routes/run_all_teach.sh --fg 08 09   # foreground + subset
#   bash routes/_common/scripts/stop_all_teach.sh  # stop everything
set -eu

SCRIPT_DIR=/workspace/simulation/isaac/routes/_common/scripts
ORCH=$SCRIPT_DIR/run_all_teach.sh
STOP=$SCRIPT_DIR/stop_all_teach.sh

# optional --fg flag to run in foreground
MODE=bg
ARGS=()
for a in "$@"; do
    if [ "$a" = "--fg" ]; then
        MODE=fg
    else
        # allow passing short names (e.g. "06" expands to "06_nw_ne")
        case "$a" in
            04) ARGS+=(04_nw_se) ;;
            05) ARGS+=(05_ne_sw) ;;
            06) ARGS+=(06_nw_ne)  ;;
            07) ARGS+=(07_se_sw) ;;
            08) ARGS+=(08_nw_sw) ;;
            09) ARGS+=(09_se_ne) ;;
            *)  ARGS+=("$a") ;;
        esac
    fi
done

TS=$(date +%Y%m%d_%H%M%S)
LOG=/workspace/simulation/isaac/routes/_common/runs/run_all_teach_$TS.log
mkdir -p "$(dirname "$LOG")"

echo "=== run_all_teach ==="
echo "  routes: ${ARGS[*]:-04..09 (all)}"
echo "  mode:   $MODE"
echo "  log:    $LOG"
echo "  stop:   bash $STOP"
echo "  tail:   tail -F $LOG"
echo "  summary: /tmp/run_all_teach_summary.txt"
echo ""

if [ "$MODE" = "fg" ]; then
    exec bash "$ORCH" "${ARGS[@]}"
else
    nohup bash "$ORCH" "${ARGS[@]}" >"$LOG" 2>&1 &
    ORCH_PID=$!
    disown "$ORCH_PID" || true
    echo "orchestrator pid: $ORCH_PID  ->  $LOG"
    # keep a pointer for later convenience
    echo "$ORCH_PID" > /tmp/run_all_teach.pid
    echo "$LOG"      > /tmp/run_all_teach.log_path
fi
