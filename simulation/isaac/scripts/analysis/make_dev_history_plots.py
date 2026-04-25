#!/usr/bin/env python3
"""rebuild milestone-experiment plots in the thesis-consistent style

writes per-experiment PNGs into
    /workspace/simulation/isaac/experiments/<NN>/results/
using the same plot_trajectory_map template as the route plots so all
the development-history figures match Chapter 5 Phase-2 visual language.

source CSVs are pulled from /root/isaac_archive/experiments/ (where the
old data lived) or from /workspace/simulation/isaac/experiments/ when
the experiment kept its raw logs.
"""
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
from plot_trajectory_map import plot_trajectory_map

ROOT = Path('/workspace/simulation/isaac/experiments')
ARC = Path('/root/isaac_archive/experiments')


def _ensure(out):
    out.parent.mkdir(parents=True, exist_ok=True)


def _peek_columns(p):
    if not Path(p).is_file():
        return []
    with open(p) as f:
        return next(csv.reader(f))


def _has(p, col):
    return col in _peek_columns(p)


def _xrange(csvs, x_col='x', y_col='y'):
    """quick stat: count + path-length"""
    xs, ys = [], []
    for p in csvs:
        if not Path(p).is_file(): continue
        with open(p) as f:
            for r in csv.DictReader(f):
                try:
                    xs.append(float(r[x_col])); ys.append(float(r[y_col]))
                except: continue
    if not xs: return 0, 0.0
    xs = np.array(xs); ys = np.array(ys)
    return len(xs), float(np.hypot(np.diff(xs), np.diff(ys)).sum())


# per-experiment plot builders

def exp_02():
    src = ARC / '02_slam_rgbd_only' / 'results' / 'trajectory.csv'
    out = ROOT / '02_slam_rgbd_only' / 'results' / 'thesis_traj.png'
    if not src.is_file(): return
    n, plen = _xrange([src], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(src), 'label': 'GT trajectory (RGB-D SLAM mapping mode)',
             'color': '#dc2626', 'linewidth': 2.0,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 02 RGB-D SLAM mapping mode  |  road, baseline (no atlas, no IMU)',
        metrics_lines=[
            'exp 02  RGB-D SLAM mapping mode',
            f'samples : {n}',
            f'GT path : {plen:.0f} m',
            'outcome : 58 % completion, 5-10 m lateral drift',
        ],
        with_obstacles=False, with_waypoints=False,
        route='road', xlim=(-115, 90), ylim=(-30, 20), figsize=(15, 6.5),
    )


def exp_03():
    src = ARC / '03_slam_localization' / 'results' / 'trajectory.csv'
    out = ROOT / '03_slam_localization' / 'results' / 'thesis_traj.png'
    if not src.is_file(): return
    n, plen = _xrange([src], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(src),
             'label': 'GT trajectory (RGB-D SLAM localization mode + atlas)',
             'color': '#dc2626', 'linewidth': 2.0,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 03 RGB-D SLAM localization + pre-built atlas  |  road',
        metrics_lines=[
            'exp 03  RGB-D SLAM localization',
            f'samples : {n}',
            f'GT path : {plen:.0f} m',
            'outcome : 60 %, 1.7 % lost frames, drift after bypass',
        ],
        with_obstacles=False, with_waypoints=False,
        route='road', xlim=(-115, 90), ylim=(-30, 20), figsize=(15, 6.5),
    )


def exp_27():
    src = ARC / '27_fullpath_lateral_snap' / 'results' / 'exp27_trajectory.csv'
    out = ROOT / '27_fullpath_lateral_snap' / 'results' / 'thesis_traj.png'
    if not src.is_file(): return
    n, plen = _xrange([src], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(src), 'label': 'GT (silent SLAM stall: pose froze, GT drifted)',
             'color': '#dc2626', 'linewidth': 2.0,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 27 full-path lateral snap  |  road, silent SLAM tracking failure',
        metrics_lines=[
            'exp 27  full-path lateral snap',
            f'samples : {n}',
            f'GT path : {plen:.0f} m',
            'outcome : 44 %; SLAM pose frozen at (43.6, -3.1)',
        ],
        with_obstacles=False, with_waypoints=False,
        route='road', xlim=(-115, 90), ylim=(-30, 20), figsize=(15, 6.5),
    )


def exp_30():
    """pick the best teach_repeat or odom_imu run: largest x-extent.  the
    files have GT + odom side-by-side which lets us overlay drift visually
    """
    candidates = sorted((ARC / '30_teach_and_repeat' / 'logs').glob(
        'odom_imu_south_*.csv'))
    best = None; best_dist = -1e9
    for c in candidates[-20:]:
        try:
            with open(c) as f:
                rd = csv.DictReader(f)
                xs, ys = [], []
                for r in rd:
                    try:
                        xs.append(float(r['gt_x'])); ys.append(float(r['gt_y']))
                    except: continue
            if not xs: continue
            d = max(xs) - min(xs)
            if d > best_dist:
                best_dist = d; best = c
        except Exception: continue
    if best is None: return
    out = ROOT / '30_teach_and_repeat' / 'results' / 'thesis_traj.png'
    n, plen = _xrange([best], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(best),
             'label': 'odom + IMU + ORB anchors (what the pipeline thought)',
             'color': '#f97316', 'linewidth': 1.8, 'linestyle': '--',
             'x_col': 'odom_x', 'y_col': 'odom_y'},
            {'csv': str(best),
             'label': 'GT (actual robot path)',
             'color': '#dc2626', 'linewidth': 2.0,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 30 first teach-and-repeat prototype  |  south, encoder + ORB anchors vs GT',
        metrics_lines=[
            'exp 30  T&R prototype',
            f'samples : {n}',
            f'GT path : {plen:.0f} m',
            'outcome : 51 m / 196 m (26 %)',
            'odom drift 1.3 - 1.8 m',
        ],
        with_obstacles=False, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 8.5),
    )


def exp_35():
    src_no = sorted((ROOT / '35_road_obstacle_avoidance' / 'logs').glob(
        'road_noobs_*.csv'))
    src_obs = sorted((ROOT / '35_road_obstacle_avoidance' / 'logs').glob(
        'road_obs_*.csv'))
    if not src_no and not src_obs: return
    out = ROOT / '35_road_obstacle_avoidance' / 'results' / 'thesis_traj.png'
    _ensure(out)
    trajs = []
    if src_no:
        trajs.append({'csv': str(src_no[-1]),
                      'label': 'baseline (no obstacles, 100 % complete)',
                      'color': '#16a34a', 'linewidth': 2.0,
                      'x_col': 'gt_x', 'y_col': 'gt_y'})
    if src_obs:
        trajs.append({'csv': str(src_obs[-1]),
                      'label': 'with cones + tent  (stuck on cone group 1)',
                      'color': '#dc2626', 'linewidth': 2.0,
                      'x_col': 'gt_x', 'y_col': 'gt_y'})
    plot_trajectory_map(
        trajectories=trajs,
        output=str(out),
        title='exp 35 road obstacle avoidance  |  reactive gap-nav baseline',
        metrics_lines=[
            'exp 35  road obstacle avoidance',
            'baseline : 167 m, 100 %, 0.62 m/s',
            'with obs : 25 % (stuck on cone group 1)',
            'failure  : 0.4 m gap < 0.67 m robot width',
        ],
        with_obstacles=True, with_waypoints=False,
        route='road', xlim=(-115, 90), ylim=(-30, 20), figsize=(15, 6.5),
    )


def exp_41():
    src = ARC / '41_trajectory_follow' / 'logs' / 'exp41_roundtrip_outbound_traj.csv'
    if not src.is_file():
        cands = list((ARC / '41_trajectory_follow' / 'logs').glob('isaac_trajectory*.csv'))
        if cands: src = cands[0]
    if not Path(src).is_file(): return
    out = ROOT / '41_trajectory_follow' / 'results' / 'thesis_traj.png'
    cols = _peek_columns(src)
    x_col = 'gt_x' if 'gt_x' in cols else ('x' if 'x' in cols else None)
    y_col = 'gt_y' if 'gt_y' in cols else ('y' if 'y' in cols else None)
    if x_col is None: return
    n, plen = _xrange([src], x_col, y_col)
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(src), 'label': 'GT (Nav2 + GT localization, route memory)',
             'color': '#16a34a', 'linewidth': 2.0,
             'x_col': x_col, 'y_col': y_col},
        ],
        output=str(out),
        title='exp 41 T&R + Nav2 + GT localization  |  proves planning works clean',
        metrics_lines=[
            'exp 41  T&R + Nav2 + GT loc',
            f'samples : {n}',
            f'GT path : {plen:.0f} m',
            'outcome : 43/43 WPs no obs (100 %)',
            '          41/43 with 4 barriers (95 %)',
        ],
        with_obstacles=True, with_waypoints=False,
        route='road', xlim=(-115, 90), ylim=(-30, 20), figsize=(15, 6.5),
    )


def exp_49():
    gt = ARC / '49_nav2_vio_roundtrip' / 'logs' / 'exp49_gt_traj.csv'
    vio = ARC / '49_nav2_vio_roundtrip' / 'logs' / 'exp49_vio_traj.csv'
    out = ROOT / '49_nav2_vio_roundtrip' / 'results' / 'thesis_traj.png'
    if not gt.is_file() or not vio.is_file(): return
    n, plen = _xrange([gt], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(gt), 'label': 'GT trajectory (Isaac Sim)',
             'color': '#0f172a', 'linewidth': 1.6, 'linestyle': '--',
             'x_col': 'gt_x', 'y_col': 'gt_y'},
            {'csv': str(vio), 'label': 'ORB-SLAM3 VIO (raw, no anchor fusion)',
             'color': '#f97316', 'linewidth': 1.8,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 49 first full Nav2 + VIO round-trip  |  ATE 0.534 m / 395 m',
        metrics_lines=[
            'exp 49  Nav2 + VIO round-trip',
            f'GT path : {plen:.0f} m',
            'ATE     : 0.534 m RMSE',
            'WPs     : 81/91  (89 %)',
            'lost frames: 0',
        ],
        with_obstacles=False, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 8.5),
    )


def exp_55():
    src = ARC / '55_visual_teach_repeat' / 'results' / \
          'run3_success_outbound_drift_return' / 'trajectory.csv'
    if not src.is_file(): return
    out = ROOT / '55_visual_teach_repeat' / 'results' / 'thesis_traj.png'
    n, plen = _xrange([src], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(src), 'label': 'nav-pose (matcher PnP-RANSAC corrections)',
             'color': '#f97316', 'linewidth': 1.6, 'linestyle': '--',
             'x_col': 'nav_x', 'y_col': 'nav_y'},
            {'csv': str(src), 'label': 'GT trajectory',
             'color': '#dc2626', 'linewidth': 2.0,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 55 visual landmark matcher (PnP-RANSAC anchors)  |  KEY INNOVATION',
        metrics_lines=[
            'exp 55  visual landmark matcher',
            f'GT path : {plen:.0f} m',
            'WPs     : 19/94  (20 %)',
            'match rate: 13 %',
            'drift   : mean 4.98 m / max 18.69 m',
        ],
        with_obstacles=True, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 8.5),
    )


def exp_56():
    src = ARC / '56_projection_cap_1m' / 'results' / 'repeat_run' / 'trajectory.csv'
    if not src.is_file(): return
    out = ROOT / '56_projection_cap_1m' / 'results' / 'thesis_traj.png'
    n, plen = _xrange([src], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(src), 'label': 'nav-pose (matcher + 1 m projection cap)',
             'color': '#f97316', 'linewidth': 1.6, 'linestyle': '--',
             'x_col': 'nav_x', 'y_col': 'nav_y'},
            {'csv': str(src), 'label': 'GT trajectory',
             'color': '#dc2626', 'linewidth': 2.0,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 56 WP projection cap 1 m  |  2.9× longer run than exp 55',
        metrics_lines=[
            'exp 56  projection cap 1 m',
            f'GT path : {plen:.0f} m',
            'WPs     : 83/94  (88 %)',
            'drift max : 8.61 m  (vs 18.69 in exp 55)',
            'loop    : nearly closed',
        ],
        with_obstacles=True, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 8.5),
    )


def exp_58():
    src = ARC / '58_route_sanitize_accum' / 'results' / 'repeat_run' / 'trajectory.csv'
    if not src.is_file(): return
    out = ROOT / '58_route_sanitize_accum' / 'results' / 'thesis_traj.png'
    n, plen = _xrange([src], 'gt_x', 'gt_y')
    _ensure(out)
    plot_trajectory_map(
        trajectories=[
            {'csv': str(src), 'label': 'nav-pose (drift-reinforcement bug)',
             'color': '#f97316', 'linewidth': 1.6, 'linestyle': '--',
             'x_col': 'nav_x', 'y_col': 'nav_y'},
            {'csv': str(src), 'label': 'GT trajectory',
             'color': '#dc2626', 'linewidth': 2.0,
             'x_col': 'gt_x', 'y_col': 'gt_y'},
        ],
        output=str(out),
        title='exp 58 running landmark accumulator  |  drift-reinforcement bug (caught)',
        metrics_lines=[
            'exp 58  accumulator bug',
            f'GT path : {plen:.0f} m',
            'failure : new landmark recorded at drifted pose',
            '          → next match latched on, drift grew',
            'peak drift: 80+ m  →  feature disabled in exp 59',
        ],
        with_obstacles=True, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 8.5),
    )


def exp_60():
    gt = ARC / '60_final_approach_10cm' / 'results' / 'repeat_run' / 'traj_gt.csv'
    sl = ARC / '60_final_approach_10cm' / 'results' / 'repeat_run' / 'traj_slam.csv'
    if not gt.is_file(): return
    out = ROOT / '60_final_approach_10cm' / 'results' / 'thesis_traj.png'
    n, plen = _xrange([gt], 'x', 'y')
    _ensure(out)
    trajs = []
    if sl.is_file():
        trajs.append({'csv': str(sl),
                      'label': 'SLAM nav-pose (matcher + 10 cm finisher)',
                      'color': '#f97316', 'linewidth': 1.6, 'linestyle': '--',
                      'x_col': 'x', 'y_col': 'y'})
    trajs.append({'csv': str(gt),
                  'label': 'GT (actual robot path)',
                  'color': '#dc2626', 'linewidth': 2.0,
                  'x_col': 'x', 'y_col': 'y'})
    plot_trajectory_map(
        trajectories=trajs,
        output=str(out),
        title='exp 60 precise final-WP finisher  |  south, 10 cm tolerance + final-5 no-skip',
        metrics_lines=[
            'exp 60  precise final-approach',
            f'GT path : {plen:.0f} m',
            'finisher: 10 cm tolerance on last WP',
            'rule    : final 5 WPs never SKIP',
            'loop closure: << 0.5 m',
        ],
        with_obstacles=True, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 8.5),
    )


def exp_62():
    gt = ARC / '62_tight_detour_anchor_sanity' / 'results' / 'repeat_run' / 'traj_gt.csv'
    sl = ARC / '62_tight_detour_anchor_sanity' / 'results' / 'repeat_run' / 'traj_slam.csv'
    if not gt.is_file(): return
    out = ROOT / '62_tight_detour_anchor_sanity' / 'results' / 'thesis_traj.png'
    n, plen = _xrange([gt], 'x', 'y')
    _ensure(out)
    trajs = []
    if sl.is_file():
        trajs.append({'csv': str(sl),
                      'label': 'SLAM nav-pose (anchor sanity gate 3 m)',
                      'color': '#f97316', 'linewidth': 1.6, 'linestyle': '--',
                      'x_col': 'x', 'y_col': 'y'})
    trajs.append({'csv': str(gt),
                  'label': 'GT (actual robot path)',
                  'color': '#dc2626', 'linewidth': 2.0,
                  'x_col': 'x', 'y_col': 'y'})
    plot_trajectory_map(
        trajectories=trajs,
        output=str(out),
        title='exp 62 anchor sanity gate  |  south, consistency 10 m → 3 m',
        metrics_lines=[
            'exp 62  anchor sanity gate',
            f'GT path : {plen:.0f} m',
            'consistency: |anchor - vio| < 3 m  (was 10 m)',
            'rejects wrong-building matches in repetitive scenes',
        ],
        with_obstacles=True, with_waypoints=False,
        route='south', xlim=(-115, 90), ylim=(-55, 55), figsize=(15, 8.5),
    )


def exp_70():
    src_b = ROOT / '70_imu_fidelity' / 'baseline' / 'traj_gt.csv'
    src_a = ROOT / '70_imu_fidelity' / 'run_A_accel_noise' / 'traj_gt.csv'
    if not src_b.is_file(): return
    out = ROOT / '70_imu_fidelity' / 'results' / 'thesis_traj.png'
    cols = _peek_columns(src_b)
    x_col = 'x' if 'x' in cols else 'gt_x'
    y_col = 'y' if 'y' in cols else 'gt_y'
    trajs = [
        {'csv': str(src_b), 'label': 'GT (baseline IMU noise model, drift max 0.74 m)',
         'color': '#dc2626', 'linewidth': 2.0,
         'x_col': x_col, 'y_col': y_col},
    ]
    if src_a.is_file():
        trajs.append({
            'csv': str(src_a),
            'label': 'GT (Run A: accel noise injected, drift max 0.69 m, -5 %)',
            'color': '#16a34a', 'linewidth': 1.8, 'linestyle': '--',
            'x_col': x_col, 'y_col': y_col})
    _ensure(out)
    plot_trajectory_map(
        trajectories=trajs,
        output=str(out),
        title='exp 70 IMU fidelity ablation  |  noise model is realistic, not load-bearing',
        metrics_lines=[
            'exp 70  IMU fidelity ablation',
            'baseline drift max : 0.74 m',
            'run A (accel noise): 0.69 m  (-5 %)',
            'conclusion: scale, not noise, drives drift',
        ],
        with_obstacles=False, with_waypoints=False,
        route='road', xlim=(-115, 90), ylim=(-30, 20), figsize=(15, 6.5),
    )


def exp_73():
    """exp 73 was originally on 09_se_ne (101 m driven, stuck early).
    The exp 74 rerun on 01_road actually got 163 m down the road before
    stalling - a more legible failure visual on a familiar route.
    """
    src = Path('/root/isaac_tr_datasets/01_road/baseline_stock_nav2/traj_gt.csv')
    teach = Path('/root/isaac_tr_datasets/01_road/teach/teach_outputs/traj_gt_world.csv')
    if not src.is_file(): return
    out = ROOT / '73_stock_nav2_baseline' / 'results' / 'thesis_traj.png'
    n, plen = _xrange([src], 'x', 'y')
    _ensure(out)
    trajs = []
    if teach.is_file():
        trajs.append({'csv': str(teach),
                      'label': 'teach GT (reference, +-321 m round-trip)',
                      'color': '#0f172a', 'linewidth': 1.4, 'linestyle': '--',
                      'x_col': 'x', 'y_col': 'y'})
    trajs.append({'csv': str(src),
                  'label': 'GT (stock Nav2: stalled in inflation, no matcher)',
                  'color': '#dc2626', 'linewidth': 2.4,
                  'x_col': 'x', 'y_col': 'y'})
    plot_trajectory_map(
        trajectories=trajs,
        output=str(out),
        title='exp 73/74 stock Nav2 baseline  |  road, drone drove 163 m before stall',
        metrics_lines=[
            'exp 73/74  stock Nav2 baseline',
            f'samples : {n}',
            f'GT path : {plen:.0f} m  (out of +-321 m round-trip)',
            'WPs     : 26/80  (33 % coverage)',
            'reach   : 56.1 m short of apex',
            'failure : RPP collision loop in tree-inflation',
        ],
        with_obstacles=True, with_waypoints=False,
        route='road', xlim=(-115, 90), ylim=(-30, 20), figsize=(15, 6.5),
    )


def main():
    builders = [exp_02, exp_03, exp_27, exp_30, exp_35, exp_41, exp_49,
                exp_55, exp_56, exp_58, exp_60, exp_62, exp_70, exp_73]
    for b in builders:
        try:
            b()
        except Exception as e:
            print(f'  [warn] {b.__name__}: {e}')
    print('done')


if __name__ == '__main__':
    main()
