#!/usr/bin/env python3
"""Early-abort watchdog for a baseline run.

Monitors four conditions and writes `/tmp/baseline_abort.txt` when any
one trips:

  (1) Process heartbeat - comma-separated PID list must all still be alive.
  (2) GT stall - robot position from /tmp/isaac_pose.txt hasn't moved more
      than GT_STALL_MIN_M across any GT_STALL_WINDOW_S sliding window
      (after WARMUP_S grace).
  (3) WP-progress stall - goals.log (client log) hasn't emitted a new
      'REACHED' line in NO_WP_MAX_S seconds (after WARMUP_S grace).
  (4) RESULT seen - RESULT: ... line in goals.log (success path).  This
      one writes /tmp/baseline_done.txt (not abort) so wait-loop exits OK.

The wait-loop in run.sh tails /tmp/baseline_abort.txt and /tmp/baseline_done.txt
alongside the existing RESULT grep.
"""
import argparse, math, os, time

ABORT_PATH = '/tmp/baseline_abort.txt'
DONE_PATH  = '/tmp/baseline_done.txt'


def read_gt():
    try:
        with open('/tmp/isaac_pose.txt') as f:
            t = f.readline().strip().split()
            return float(t[0]), float(t[1])
    except Exception:
        return None


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def last_reached_ts(goals_log):
    try:
        with open(goals_log) as f:
            ts = None
            for line in f:
                if 'REACHED' in line or 'RESULT: reached' in line:
                    # line format [INFO] [1776891234.567]...
                    a = line.find('[', line.find('[') + 1)  # 2nd '['
                    b = line.find(']', a + 1)
                    if a != -1 and b != -1:
                        try:
                            ts = float(line[a+1:b])
                        except ValueError:
                            pass
            return ts
    except Exception:
        return None


def has_result(goals_log):
    try:
        with open(goals_log) as f:
            return any('RESULT:' in line for line in f)
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pids', required=True, help='comma-separated critical pids')
    ap.add_argument('--goals-log', required=True)
    ap.add_argument('--warmup-s', type=float, default=240.0)
    ap.add_argument('--gt-stall-window-s', type=float, default=180.0)
    ap.add_argument('--gt-stall-min-m', type=float, default=1.5)
    # 0 disables the no-WP-REACH check (relies on GT stall only -
    # required for baselines where SLAM drift blocks goal_checker firing).
    ap.add_argument('--no-wp-max-s', type=float, default=0.0)
    ap.add_argument('--tick-s', type=float, default=10.0)
    args = ap.parse_args()

    pids = [int(p) for p in args.pids.split(',') if p]
    try: os.remove(ABORT_PATH)
    except FileNotFoundError: pass
    try: os.remove(DONE_PATH)
    except FileNotFoundError: pass

    start = time.time()
    gt_hist = []  # [(t, x, y)]

    def abort(reason):
        with open(ABORT_PATH, 'w') as f:
            f.write(reason + '\n')
        print(f'[watchdog] ABORT: {reason}')

    def done(reason):
        with open(DONE_PATH, 'w') as f:
            f.write(reason + '\n')
        print(f'[watchdog] DONE: {reason}')

    while True:
        now = time.time()
        elapsed = now - start

        # (4) RESULT - quickest path, exit watchdog
        if has_result(args.goals_log):
            done('RESULT line seen in goals.log')
            return 0

        # (1) heartbeat
        dead = [p for p in pids if not alive(p)]
        if dead:
            abort(f'critical process(es) dead: {dead}')
            return 1

        # (2) GT stall
        gt = read_gt()
        if gt is not None:
            gt_hist.append((now, gt[0], gt[1]))
            # prune older than window
            gt_hist = [(t, x, y) for (t, x, y) in gt_hist
                       if now - t <= args.gt_stall_window_s]
            if elapsed > args.warmup_s and len(gt_hist) >= 6:
                dx = max(x for _, x, _ in gt_hist) - min(x for _, x, _ in gt_hist)
                dy = max(y for _, _, y in gt_hist) - min(y for _, _, y in gt_hist)
                disp = math.hypot(dx, dy)
                if disp < args.gt_stall_min_m:
                    abort(f'GT stall - moved {disp:.2f} m in '
                          f'{args.gt_stall_window_s:.0f} s (<{args.gt_stall_min_m} m)')
                    return 1

        # (3) WP-progress stall - only if --no-wp-max-s > 0
        if args.no_wp_max_s > 0 and elapsed > args.warmup_s:
            last = last_reached_ts(args.goals_log)
            if last is None:
                if elapsed > args.warmup_s + args.no_wp_max_s:
                    abort(f'no WP REACHED after {int(elapsed)} s')
                    return 1
            else:
                since = time.time() - last
                if since > args.no_wp_max_s:
                    abort(f'no new WP REACHED in last {int(since)} s')
                    return 1

        time.sleep(args.tick_s)


if __name__ == '__main__':
    raise SystemExit(main())
