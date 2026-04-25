#!/usr/bin/env python3
"""generator for results/final/*.png thesis figures (current campaign)

reads routes/_common/metrics.json + per-route traj_gt CSVs from
/root/isaac_tr_datasets/<route>/{teach,repeat,baseline_stock_nav2,
baseline_rgbd_no_imu}/.  writes a small curated set of thesis figures to
results/final/

replaces the old 04/05/06 plots (those were pre-campaign legacy)

run:  python3 scripts/analysis/make_thesis_final_plots.py
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
from plot_trajectory_map import plot_trajectory_map

ROOT = Path('/workspace/simulation/isaac')
OUT = ROOT / 'results' / 'final'
OUT.mkdir(parents=True, exist_ok=True)

DATASETS = Path('/root/isaac_tr_datasets')

ROUTES = ['01_road', '02_north_forest', '03_south',
          '04_nw_se', '05_ne_sw', '06_nw_ne',
          '07_se_sw', '08_nw_sw', '09_se_ne']

ROUTE_LABELS = {
    '01_road':         'road',
    '02_north_forest': 'N-forest',
    '03_south':        'south',
    '04_nw_se':        'NW-SE',
    '05_ne_sw':        'NE-SW',
    '06_nw_ne':        'NW-NE',
    '07_se_sw':        'SE-SW',
    '08_nw_sw':        'NW-SW',
    '09_se_ne':        'SE-NE',
}

ROUTE_COLORS = {
    '01_road':         '#1f77b4',
    '02_north_forest': '#2ca02c',
    '03_south':        '#d62728',
    '04_nw_se':        '#9467bd',
    '05_ne_sw':        '#8c564b',
    '06_nw_ne':        '#e377c2',
    '07_se_sw':        '#7f7f7f',
    '08_nw_sw':        '#bcbd22',
    '09_se_ne':        '#17becf',
}

COL_OURS  = '#f97316'   # orange
COL_STOCK = '#2563eb'   # blue
COL_RGBD  = '#dc2626'   # red


def load_metrics():
    with open(ROOT / 'routes' / '_common' / 'metrics.json') as f:
        return json.load(f)


# figure 04: all 9 teach-GT trajectories on scene map

def plot_04_teach_9routes():
    trajs = []
    for r in ROUTES:
        p = DATASETS / r / 'teach' / 'teach_outputs' / 'traj_gt_world.csv'
        if p.is_file():
            trajs.append({
                'csv': str(p),
                'label': f'{ROUTE_LABELS[r]}',
                'color': ROUTE_COLORS[r],
                'x_col': 'x', 'y_col': 'y',
                'linewidth': 1.6,
            })
    if not trajs:
        print('04: no teach trajectories found')
        return
    plot_trajectory_map(
        trajectories=trajs,
        output=str(OUT / '04_teach_gt_9routes.png'),
        title='Teach GT trajectories - all 9 routes on forest scene',
        metrics_lines=None,
        with_obstacles=False,
        with_waypoints=False,
        route='south',
        xlim=(-115, 90), ylim=(-55, 55),
        figsize=(16, 10),
    )
    print('04_teach_gt_9routes.png')


# figure 05: drift mean / p95 / max, 3 stacks side by side per route

def plot_05_drift_bars(metrics):
    fig, ax = plt.subplots(figsize=(14, 5.5))
    x = np.arange(len(ROUTES))
    w = 0.27
    methods = [('our custom', 'our custom T&R', COL_OURS),
               ('exp 74 stock', 'exp 74 stock Nav2', COL_STOCK),
               ('exp 76 RGB-D', 'exp 76 RGB-D (no IMU)', COL_RGBD)]
    for i, (key, lab, col) in enumerate(methods):
        means = [metrics.get(r, {}).get(key, {}).get('drift_mean', 0) or 0 for r in ROUTES]
        p95s  = [metrics.get(r, {}).get(key, {}).get('drift_p95',  0) or 0 for r in ROUTES]
        offs = (i - 1) * w
        ax.bar(x + offs, means, w, color=col, label=lab, alpha=0.85, edgecolor='black', linewidth=0.3)
        ax.errorbar(x + offs, means, yerr=[np.zeros(len(means)), np.array(p95s) - np.array(means)],
                    fmt='none', ecolor='#222', elinewidth=0.8, capsize=2.5, alpha=0.7)
    ax.set_xticks(x)
    ax.set_xticklabels([ROUTE_LABELS[r] for r in ROUTES], rotation=20)
    ax.set_ylabel('localization drift |nav - GT| (m)')
    ax.set_title('drift mean (bars) + p95 (error bars) by route and stack')
    ax.axhline(10, color='red', ls=':', lw=0.8, alpha=0.6, label='10 m endpoint tolerance')
    ax.legend(loc='upper left', fontsize=9, ncol=2)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '05_drift_bars.png', dpi=130)
    plt.close()
    print('05_drift_bars.png')


# figure 07: 3-stack trajectory comparison on a chosen route

def _compare_route(route, out_name, title_suffix):
    our = DATASETS / route / 'repeat' / 'results' / 'repeat_run'
    e74 = DATASETS / route / 'baseline_stock_nav2'
    e76 = DATASETS / route / 'baseline_rgbd_no_imu'
    teach = DATASETS / route / 'teach' / 'teach_outputs' / 'traj_gt_world.csv'
    #build metric box
    metrics = load_metrics().get(route, {})

    def _fmt(stack_label, key):
        d = metrics.get(key, {})
        if not d:
            return f'{stack_label:<12}: n/a'
        cov = d.get('cov_pct', 0) or 0
        cov_v = d.get('cov_visited', 0); cov_t = d.get('cov_total', 0)
        reach = d.get('final_d', 0) or 0
        ret = d.get('return_d', 0) or 0
        reach_ok = '✓' if reach <= 10 else '✗'
        ret_ok = '✓' if ret <= 10 else '✗'
        return (f'{stack_label:<12}: cov {cov_v}/{cov_t} ({cov:.0f}%)  '
                f'reach {reach:.1f}m {reach_ok}  return {ret:.1f}m {ret_ok}')
    metrics_lines = [
        _fmt('our T&R',   'our custom'),
        _fmt('exp 74 stk', 'exp 74 stock'),
        _fmt('exp 76 RGB', 'exp 76 RGB-D'),
    ]

    trajs = []
    if teach.is_file():
        trajs.append({'csv': str(teach), 'label': 'teach GT (reference)',
                      'color': '#111827', 'x_col': 'x', 'y_col': 'y',
                      'linewidth': 1.3, 'linestyle': '--'})
    for src, lab, col, lw in [
        (our / 'traj_gt.csv',           'our custom T&R',      COL_OURS,  2.4),
        (e74 / 'traj_gt.csv',           'exp 74 stock Nav2',    COL_STOCK, 1.8),
        (e76 / 'traj_gt.csv',           'exp 76 RGB-D (no IMU)',COL_RGBD,  1.8),
    ]:
        if src.is_file() and src.stat().st_size > 50:
            trajs.append({'csv': str(src), 'label': lab, 'color': col,
                          'x_col': 'x', 'y_col': 'y', 'linewidth': lw})
    if len(trajs) < 2:
        print(f'{out_name}: not enough data'); return
    # 01_road's obstacles live in plot_trajectory_map's hardcoded BARRIERS   
    # (legacy-shape cone walls + tent), activated when route='road'   
    # 04-09 live in spawn_obstacles.OBSTACLES[route].
    route_arg = 'road' if route == '01_road' else route
    plot_trajectory_map(
        trajectories=trajs,
        output=str(OUT / out_name),
        title=f'{ROUTE_LABELS[route]} - 3-stack comparison {title_suffix}',
        metrics_lines=metrics_lines,
        with_obstacles=True,
        with_waypoints=False,
        route=route_arg,
        xlim=(-115, 90), ylim=(-55, 55),
        figsize=(16, 10),
    )
    print(out_name)


def main():
    metrics = load_metrics()
    plot_04_teach_9routes()
    plot_05_drift_bars(metrics)
    _compare_route('09_se_ne', '07_compare_09_se_ne.png', '(100 % headline)')
    _compare_route('04_nw_se', '08_compare_04_nw_se.png', '(longest diagonal, 1223 m)')
    _compare_route('01_road',  '09_compare_01_road.png',  '(road loop, 17 cones + tent)')
    print('done')


if __name__ == '__main__':
    main()
