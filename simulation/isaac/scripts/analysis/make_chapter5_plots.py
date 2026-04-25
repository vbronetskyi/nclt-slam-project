#!/usr/bin/env python3   
"""thesis chapter 5 figures: trajectory plots for routes 12/13 + drift
comparison curves.  fills the gaps that plot_three_way.py + make_thesis_final
didn't cover (those only handled 01-09)
"""
import csv
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
from plot_trajectory_map import plot_trajectory_map

ROOT = Path('/workspace/simulation/isaac')
OUT = ROOT / 'results' / 'final' / 'phase2'
OUT.mkdir(parents=True, exist_ok=True)
DATASETS = Path('/root/isaac_tr_datasets')

COL_OURS  = '#f97316'
COL_TEACH = '#111827'


def _read_xy(p, x_col='x', y_col='y'):
    if not p.is_file(): return None, None
    xs, ys = [], []
    with open(p) as f:
        rd = csv.DictReader(f)
        for row in rd:
            try:
                xs.append(float(row[x_col])); ys.append(float(row[y_col]))
            except (ValueError, KeyError):
                continue
    return np.array(xs), np.array(ys)


EXTRA_15 = {
    '10_nmid_smid': {'final_d': 4.2, 'return_d': 4.8, 'drift_mean': 3.0,
                     'cov_pct': 82.5, 'cov_visited': 33, 'cov_total': 40},
    '11_nw_mid':    {'final_d': 3.1, 'return_d': 5.2, 'drift_mean': 2.0,
                     'cov_pct': 79.5, 'cov_visited': 35, 'cov_total': 44},
    '12_ne_mid':    {'final_d': 1.1, 'return_d': 11.8, 'drift_mean': 5.2,
                     'cov_pct': 82.6, 'cov_visited': 38, 'cov_total': 46},
    '13_cross_nws': {'final_d': 2.6, 'return_d': 28.7, 'drift_mean': 18.8,
                     'cov_pct': 60.5, 'cov_visited': 23, 'cov_total': 38},
    '14_se_mid':    {'final_d': 3.7, 'return_d': 2.7, 'drift_mean': 2.6,
                     'cov_pct': 28.2, 'cov_visited': 11, 'cov_total': 39},
    '15_wmid_smid': {'final_d': 4.8, 'return_d': 6.5, 'drift_mean': 7.2,
                     'cov_pct': 50.0, 'cov_visited': 22, 'cov_total': 44},
}


def _route_metrics_box(route):
    """build the bottom-left metrics text for a trajectory plot"""
    import json as _json
    try:
        with open(ROOT / 'routes' / '_common' / 'metrics.json') as f:
            m = _json.load(f).get(route, {}).get('our custom', {})
    except Exception:
        m = {}
    if not m:
        m = EXTRA_15.get(route, {})
    if not m:
        return None
    cov = m.get('cov_pct', 0) or 0
    cv = m.get('cov_visited', 0); ct = m.get('cov_total', 0)
    reach = m.get('final_d', 0) or 0
    ret = m.get('return_d', 0) or 0
    drift = m.get('drift_mean', 0) or 0
    reach_ok = '✓' if reach <= 10 else '✗'
    ret_ok = '✓' if ret <= 10 else '✗'
    cov_str = f'{cv}/{ct} ({cov:.0f}%)' if ct else f'{cov:.0f}%'
    return [
        f'coverage   : {cov_str}',
        f'reach apex : {reach:.1f} m  {reach_ok}  (10 m thresh)',
        f'return spawn: {ret:.1f} m  {ret_ok}  (10 m thresh)',
        f'drift mean : {drift:.1f} m',
    ]


def fig_traj_route(route, out_name, title_extra):
    teach = DATASETS / route / 'teach' / 'teach_outputs' / 'traj_gt.csv'
    repeat = DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'traj_gt.csv'

    trajs = []
    if teach.is_file() and teach.stat().st_size > 50:
        trajs.append({'csv': str(teach), 'label': 'teach GT (reference)',
                      'color': COL_TEACH, 'x_col': 'x', 'y_col': 'y',
                      'linewidth': 1.4, 'linestyle': '--'})
    if repeat.is_file() and repeat.stat().st_size > 50:
        trajs.append({'csv': str(repeat), 'label': 'repeat GT (our T&R)',
                      'color': COL_OURS, 'x_col': 'x', 'y_col': 'y',
                      'linewidth': 2.6})
    if not trajs:
        print(f'{out_name}: no data'); return

    plot_trajectory_map(
        trajectories=trajs,
        output=str(OUT / out_name),
        title=f'{route}  |  teach vs repeat  |  {title_extra}',
        metrics_lines=_route_metrics_box(route),
        with_obstacles=True,
        with_waypoints=False,
        route=route,
        xlim=(-115, 90), ylim=(-55, 55),
        figsize=(16, 9),
    )
    print(out_name)


# drift curves: parse tf_slam.log for "err=N.Nm" entries against time stamps
def parse_drift(log_path):
    if not Path(log_path).is_file():
        return None
    err_re = re.compile(r'\[(\d+\.\d+)\].*?err=([\-\d.]+)m')
    t0 = None
    ts, errs = [], []
    for line in open(log_path):
        m = err_re.search(line)
        if not m: continue
        t = float(m.group(1)); e = float(m.group(2))
        if t0 is None: t0 = t
        ts.append(t - t0); errs.append(e)
    return np.array(ts), np.array(errs)


def fig_drift_compare_09():
    """drift over time for 09_se_ne (the 100 % headline route) across our
    custom T&R + exp 74 stock + exp 76 RGB-D-no-IMU"""
    route = '09_se_ne'
    sources = [
        ('our custom T&R', '#f97316',
         DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'tf_slam.log'),
        ('exp 74 stock Nav2', '#2563eb',
         DATASETS / route / 'baseline_stock_nav2' / 'tf_slam.log'),
        ('exp 76 RGB-D (no IMU)', '#dc2626',
         DATASETS / route / 'baseline_rgbd_no_imu' / 'tf_slam.log'),
    ]
    fig, ax = plt.subplots(figsize=(13, 5.5))
    have_any = False
    for label, col, path in sources:
        d = parse_drift(path)
        if d is None or d[0].size == 0:
            print(f'  {label}: no drift log @ {path}')
            continue
        ts, errs = d
        ax.plot(ts, errs, color=col, lw=1.6, alpha=0.9, label=label)
        have_any = True
    if not have_any:
        print('drift_curves: nothing to plot'); return
    ax.axhline(10, color='#16a34a', lw=1.0, ls='--', alpha=0.7,
               label='10 m endpoint tolerance')
    ax.axhline(3, color='#16a34a', lw=0.8, ls=':', alpha=0.55,
               label='3 m REACH tolerance')
    ax.set_xlabel('wall time since run start [s]')
    ax.set_ylabel('|published pose - GT|  [m]')
    ax.set_title(f'localization drift over time on {route}  '
                 f'(per-tick err from tf_wall_clock_relay_v55.log every 5 s)',
                 fontsize=11, fontweight='bold')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper left', fontsize=9, ncol=2, framealpha=0.92)
    plt.tight_layout()
    plt.savefig(OUT / '14_drift_curves_09.png', dpi=140)
    plt.close()
    print('14_drift_curves_09.png')


def fig_teach_only(route, out_name):
    """teach-stage trajectory plot showing GT + ORB-SLAM3 VIO (rigid-aligned
    so origin and initial heading match GT), with drift |vio_aligned − gt|
    metrics.  vio_pose_dense.csv stores VIO in its own frame (origin at
    robot spawn in SLAM coords); we align by first-pose translation +
    yaw of the first 50 samples, then drift = how much shape diverges"""
    sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
    pose = DATASETS / route / 'teach' / 'teach_outputs' / 'vio_pose_dense.csv'
    if not pose.is_file():
        print(f'{out_name}: no vio_pose_dense'); return

    import csv as _csv
    import numpy as _np
    gx, gy, vx, vy = [], [], [], []
    with open(pose) as f:
        rd = _csv.DictReader(f)
        for row in rd:
            try:
                gx.append(float(row['gt_x'])); gy.append(float(row['gt_y']))
                vx.append(float(row['vio_x'])); vy.append(float(row['vio_y']))
            except (ValueError, KeyError):
                continue
    gx, gy, vx, vy = map(_np.array, (gx, gy, vx, vy))
    if len(gx) < 100:
        print(f'{out_name}: not enough teach samples'); return

    # rigid-align VIO to GT: translate so first sample matches, rotate so
    # initial heading vector (gt[50] - gt[0]) lines up
    head = 50
    g0 = _np.array([gx[0], gy[0]]); v0 = _np.array([vx[0], vy[0]])
    gh = _np.array([gx[head], gy[head]]) - g0
    vh = _np.array([vx[head], vy[head]]) - v0
    a_g = _np.arctan2(gh[1], gh[0]); a_v = _np.arctan2(vh[1], vh[0])
    dyaw = a_g - a_v
    c, s = _np.cos(dyaw), _np.sin(dyaw)
    R = _np.array([[c, -s], [s, c]])
    V = _np.column_stack([vx - v0[0], vy - v0[1]]) @ R.T
    vx_a = V[:, 0] + g0[0]
    vy_a = V[:, 1] + g0[1]

    err = _np.hypot(vx_a - gx, vy_a - gy)
    drift_mean = err.mean()
    drift_p95 = _np.percentile(err, 95)
    drift_max = err.max()
    n = len(err); path_len = _np.hypot(_np.diff(gx), _np.diff(gy)).sum()

    # write a temporary CSV with the aligned VIO so plot_trajectory_map   
    #can pick it up via its csv loader
    import tempfile, os
    tmp = OUT / f'_tmp_{route}_vio_aligned.csv'
    with open(tmp, 'w') as f:
        f.write('x,y\n')
        for x, y in zip(vx_a, vy_a):
            f.write(f'{x:.4f},{y:.4f}\n')

    trajs = [
        {'csv': str(pose), 'label': 'teach GT (reference, +-20 Hz)',
         'color': '#0f172a', 'linewidth': 2.0,
         'x_col': 'gt_x', 'y_col': 'gt_y'},
        {'csv': str(tmp),
         'label': 'ORB-SLAM3 VIO (raw, origin-aligned, no anchor fusion)',
         'color': '#f97316', 'linewidth': 1.8, 'linestyle': '--',
         'x_col': 'x', 'y_col': 'y'},
    ]

    metrics_lines = [
        f'route        : {route}',
        f'samples      : {n}  (~{n/20:.0f} s @ 20 Hz)',
        f'GT path len  : {path_len:.0f} m',
        f'VIO drift mean : {drift_mean:.2f} m',
        f'VIO drift p95  : {drift_p95:.2f} m',
        f'VIO drift max  : {drift_max:.2f} m',
    ]
    plot_trajectory_map(
        trajectories=trajs,
        output=str(OUT / out_name),
        title=f'{route}  |  teach run: GT vs ORB-SLAM3 VIO  '
              f'(raw VIO drift mean {drift_mean:.1f} m / max {drift_max:.1f} m '
              'before anchor correction)',
        metrics_lines=metrics_lines,
        with_obstacles=False,
        with_waypoints=False,
        route=route,
        xlim=(-115, 90), ylim=(-55, 55),
        figsize=(16, 9),
    )
    try: os.unlink(tmp)
    except Exception: pass
    print(out_name)


def fig_repeat_nav_vs_gt(route, out_name):
    """repeat-stage navigation: plot GT (what the robot really did) vs the
    nav-frame pose the pipeline actually published to Nav2 (what the robot
    *thought* it was doing).  parsed from tf_slam.log lines:
        nav=(x,y) gt=(x,y) err=N.Nm regime=...
    so we get matched (nav, gt) pairs +-5 s apart with the post-fusion drift
    that the pipeline lived with during navigation
    """
    sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
    log = (DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'tf_slam.log')
    if not log.is_file():
        print(f'{out_name}: no tf_slam.log'); return
    import re, numpy as _np
    nav_re = re.compile(r'nav=\(([\-\d.]+),([\-\d.]+)\)\s*gt=\(([\-\d.]+),([\-\d.]+)\)\s*err=([\-\d.]+)m')
    nx, ny, gx, gy, e = [], [], [], [], []
    for line in open(log):
        m = nav_re.search(line)
        if not m: continue
        try:
            nx.append(float(m.group(1))); ny.append(float(m.group(2)))
            gx.append(float(m.group(3))); gy.append(float(m.group(4)))
            e.append(float(m.group(5)))
        except ValueError:
            continue
    if len(gx) < 10:
        print(f'{out_name}: not enough nav samples'); return
    nx = _np.array(nx); ny = _np.array(ny)
    gx = _np.array(gx); gy = _np.array(gy)
    err = _np.array(e); err = err[err < 50]   # drop jump transients
    drift_mean = err.mean(); drift_max = err.max()
    drift_p95 = _np.percentile(err, 95)
    path_len = _np.hypot(_np.diff(gx), _np.diff(gy)).sum()

    # write a temp CSV for plot_trajectory_map
    tmp_g = OUT / f'_tmp_{route}_repeat_gt.csv'
    tmp_n = OUT / f'_tmp_{route}_repeat_nav.csv'
    with open(tmp_g, 'w') as f:
        f.write('x,y\n')
        for x, y in zip(gx, gy): f.write(f'{x:.4f},{y:.4f}\n')
    with open(tmp_n, 'w') as f:
        f.write('x,y\n')
        for x, y in zip(nx, ny): f.write(f'{x:.4f},{y:.4f}\n')

    trajs = [
        {'csv': str(tmp_g), 'label': 'repeat GT (actual robot path)',
         'color': '#0f172a', 'linewidth': 2.0,
         'x_col': 'x', 'y_col': 'y'},
        {'csv': str(tmp_n),
         'label': 'published nav pose (what the pipeline thought)',
         'color': '#f97316', 'linewidth': 1.8, 'linestyle': '--',
         'x_col': 'x', 'y_col': 'y'},
    ]
    metrics_lines = [
        f'route        : {route}  (repeat / navigation)',
        f'tf-relay ticks: {len(gx)}  (~{len(gx)*5} s @ 0.2 Hz)',
        f'GT path len  : {path_len:.0f} m',
        f'nav drift mean: {drift_mean:.2f} m',
        f'nav drift p95 : {drift_p95:.2f} m',
        f'nav drift max : {drift_max:.2f} m',
    ]
    plot_trajectory_map(
        trajectories=trajs,
        output=str(OUT / out_name),
        title=f'{route}  |  repeat run: GT vs published nav pose  '
              f'(post-fusion drift mean {drift_mean:.1f} m / max {drift_max:.1f} m '
              'with anchor correction active)',
        metrics_lines=metrics_lines,
        with_obstacles=True,
        with_waypoints=False,
        route=route,
        xlim=(-115, 90), ylim=(-55, 55),
        figsize=(16, 9),
    )
    import os
    for p in (tmp_g, tmp_n):
        try: os.unlink(p)
        except Exception: pass
    print(out_name)


def fig_drifty_teach_success_repeat(route, out_name):
    """showcase: route where the raw VIO teach drift is large, yet the
    repeat run still reaches the apex + returns to spawn thanks to the
    matcher.  draws (1) teach GT, (2) raw VIO origin-aligned, (3) repeat
    GT, on the same scene-map.
    """
    sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
    pose = DATASETS / route / 'teach' / 'teach_outputs' / 'vio_pose_dense.csv'
    repeat_gt = DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'traj_gt.csv'
    if not pose.is_file() or not repeat_gt.is_file():
        print(f'{out_name}: missing inputs'); return

    # rigid-align VIO to GT for the teach trace
    import csv as _csv, numpy as _np, math as _math, os as _os
    gx, gy, vx, vy = [], [], [], []
    with open(pose) as f:
        rd = _csv.DictReader(f)
        for r in rd:
            try:
                gx.append(float(r['gt_x'])); gy.append(float(r['gt_y']))
                vx.append(float(r['vio_x'])); vy.append(float(r['vio_y']))
            except: continue
    gx, gy, vx, vy = map(_np.array, (gx, gy, vx, vy))
    head = 50
    g0 = _np.array([gx[0], gy[0]]); v0 = _np.array([vx[0], vy[0]])
    gh = _np.array([gx[head], gy[head]]) - g0
    vh = _np.array([vx[head], vy[head]]) - v0
    dyaw = _math.atan2(gh[1], gh[0]) - _math.atan2(vh[1], vh[0])
    c, s = _math.cos(dyaw), _math.sin(dyaw)
    R = _np.array([[c, -s], [s, c]])
    V = _np.column_stack([vx - v0[0], vy - v0[1]]) @ R.T
    vx_a = V[:, 0] + g0[0]; vy_a = V[:, 1] + g0[1]
    drift = _np.hypot(vx_a - gx, vy_a - gy)

    tmp = OUT / f'_tmp_{route}_vio.csv'
    with open(tmp, 'w') as f:
        f.write('x,y\n')
        for x, y in zip(vx_a, vy_a): f.write(f'{x:.4f},{y:.4f}\n')

    # repeat metrics
    metrics = _route_metrics_box(route) or []
    teach_metrics = [
        f'teach VIO drift mean : {drift.mean():.2f} m',
        f'teach VIO drift max  : {drift.max():.2f} m',
    ]

    trajs = [
        {'csv': str(pose), 'label': 'teach GT (reference)',
         'color': '#0f172a', 'linewidth': 1.6, 'linestyle': '--',
         'x_col': 'gt_x', 'y_col': 'gt_y'},
        {'csv': str(tmp),
         'label': 'teach raw ORB-SLAM3 VIO  (origin-aligned)',
         'color': '#f97316', 'linewidth': 1.5, 'linestyle': ':',
         'x_col': 'x', 'y_col': 'y'},
        {'csv': str(repeat_gt),
         'label': 'repeat GT (with matcher + obstacles)',
         'color': '#dc2626', 'linewidth': 2.4,
         'x_col': 'x', 'y_col': 'y'},
    ]
    plot_trajectory_map(
        trajectories=trajs,
        output=str(OUT / out_name),
        title=f'{route}  |  teach drifts but repeat reaches the goal  '
              f'(teach VIO drift max {drift.max():.1f} m  →  '
              'repeat reach + return PASS)',
        metrics_lines=teach_metrics + metrics,
        with_obstacles=True,
        with_waypoints=False,
        route=route,
        xlim=(-115, 90), ylim=(-55, 55),
        figsize=(16, 9.5),
    )
    try: _os.unlink(tmp)
    except Exception: pass
    print(out_name)


def fig_low_coverage_success(route, out_name):
    """showcase: route with low WP coverage (many waypoints skipped by the
    goal dispatcher) but reach + return both PASS.  evidence that the
    pipeline can complete the round-trip even when the matcher / obstacle
    layer makes Nav2 step over a chunk of the teach WPs."""
    sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
    teach = DATASETS / route / 'teach' / 'teach_outputs' / 'traj_gt.csv'
    repeat_gt = DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'traj_gt.csv'
    goals = DATASETS / route / 'repeat' / 'results' / 'repeat_run' / 'goals.log'
    if not teach.is_file() or not repeat_gt.is_file():
        print(f'{out_name}: missing trajectory inputs'); return

    # parse skip events from goals.log to count
    n_skip = 0; n_reached = 0
    if goals.is_file():
        for line in open(goals):
            if 'SKIP' in line: n_skip += 1
            elif 'REACH' in line.upper(): n_reached += 1

    metrics_box = _route_metrics_box(route) or []
    extra = [f'WPs skipped : {n_skip}'] if n_skip else []

    trajs = [
        {'csv': str(teach), 'label': 'teach GT (reference)',
         'color': '#0f172a', 'linewidth': 1.6, 'linestyle': '--',
         'x_col': 'x', 'y_col': 'y'},
        {'csv': str(repeat_gt), 'label': 'repeat GT (low coverage but full round-trip)',
         'color': '#dc2626', 'linewidth': 2.4,
         'x_col': 'x', 'y_col': 'y'},
    ]
    plot_trajectory_map(
        trajectories=trajs,
        output=str(OUT / out_name),
        title=f'{route}  |  many WPs skipped, full round-trip reached  '
              f'(coverage low - reach + return PASS)',
        metrics_lines=metrics_box + extra,
        with_obstacles=True,
        with_waypoints=False,
        route=route,
        xlim=(-115, 90), ylim=(-55, 55),
        figsize=(16, 9.5),
    )
    print(out_name)


def main():
    fig_traj_route(
        '12_ne_mid', '15_traj_12_ne_mid_best.png',
        'best phase-2 result (drift mean 1.4 m, both reach + return PASS)')
    fig_traj_route(
        '13_cross_nws', '16_traj_13_cross_nws_worst.png',
        'worst phase-2 result (drift mean 18.8 m, return fail at 28.7 m)')
    # 12_ne_mid: clean medium-length teach (+-165 m, 16k samples) with
    #bounded raw VIO drift (mean 2.0 m / max 4.1 m).  representative of
    # what a well-behaved teach looks like before anchor correction
    fig_teach_only('12_ne_mid', '21_teach_vio_vs_gt_12_ne_mid.png')
    #03_south: classic obstacle-bypass repeat run, drift well-behaved
    # (mean 2 m / max 3.6 m) with active matcher.  shows the navigation
    # pipeline working end-to-end on a long route + dropped obstacles
    fig_repeat_nav_vs_gt('03_south', '22_repeat_nav_vs_gt_03_south.png')
    # 04_nw_se: longest diagonal (1223 m); raw VIO drift in teach is 5-11 m
    # but matcher + Nav2 still completes round-trip (reach 7.8 m, return 5.0 m)
    fig_drifty_teach_success_repeat('04_nw_se',
                                    '23_drifty_teach_success_repeat_04_nw_se.png')
    # 11_nw_mid: only 34 % WP coverage (goal dispatcher skipped many WPs
    # adjacent to placed obstacles in the deep-NW forest segment), yet
    # reach 3.1 m + return 5.2 m PASS - same low-coverage-success pattern
    #as 14 but trajectory shape is cleaner / less ambiguous to read
    fig_low_coverage_success('11_nw_mid',
                              '24_low_coverage_success_11_nw_mid.png')
    fig_drift_compare_09()
    print('done')


if __name__ == '__main__':
    main()
