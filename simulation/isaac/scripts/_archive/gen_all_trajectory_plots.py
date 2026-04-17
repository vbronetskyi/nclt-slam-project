#!/usr/bin/env python3
import glob
import os
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

BASE = '/workspace/simulation/isaac/experiments'

# (exp_dir, title, x_col, y_col, color, summary_metrics)
CONFIGS = {
    '35_road_obstacle_avoidance': {
        'title_obs': 'Exp 35: Gap Navigator - With Obstacles (25%)',
        'title_noobs': 'Exp 35: Gap Navigator - No Obstacles (100%)',
        'color': '#9467bd',
        'metrics_obs': [
            'Method:    Gap Navigator (reactive)',
            'Result:    25% - stuck at cone group 1',
            'Issue:     drove through 1m cone gaps',
            'Speed:     0.62 m/s avg',
        ],
        'metrics_noobs': [
            'Method:    Gap Navigator (reactive)',
            'Result:    100% - road baseline',
            'Distance:  167m, 84 anchors',
            'Speed:     0.62 m/s avg',
        ],
    },
    '36_obstacle_grid': {
        'title_obs': 'Exp 36: Local Obstacle Grid - With Obstacles (25%)',
        'title_noobs': 'Exp 36: Local Obstacle Grid - No Obstacles (100%)',
        'color': '#8c564b',
        'metrics_obs': [
            'Method:   Gap Nav + depth grid',
            'Grid:     DECAY=0.95, MAX=6m, 40 sec',
            'Result:   25% - stuck at cone group 1',
            'Issue:    grid blocked entire road',
        ],
        'metrics_noobs': [
            'Method:  Gap Nav + depth grid',
            'Result:  100% baseline',
        ],
    },
    '37_grid_planner': {
        'title_obs': 'Exp 37: A* Grid Planner - With Obstacles (25%)',
        'title_noobs': 'Exp 37: A* Grid Planner - No Obstacles (100%)',
        'color': '#e377c2',
        'metrics_obs': [
            'Method:   A* on local grid',
            'Result:   25% - stuck at cone group 1',
            'Issue:    planner too slow to dodge',
        ],
        'metrics_noobs': [
            'Method:  A* on local grid',
            'Result:  100% baseline',
        ],
    },
    '38_fast_planner': {
        'title_obs': 'Exp 38: Optimized A* - With Obstacles (25%)',
        'title_noobs': 'Exp 38: Optimized A* - No Obstacles (100%)',
        'color': '#7f7f7f',
        'metrics_obs': [
            'Method:   Fast A* (scipy optimized)',
            'Result:   25% - same fail mode',
            'Insight:  planner speed not the issue',
        ],
        'metrics_noobs': [
            'Method:  Optimized A*',
            'Result:  100% baseline',
        ],
    },
    '39_hybrid_navigator': {
        'title_obs': 'Exp 39: Hybrid Gap+A* - With Obstacles (25%)',
        'title_noobs': 'Exp 39: Hybrid Gap+A* - No Obstacles (100%)',
        'color': '#bcbd22',
        'metrics_obs': [
            'Method:  Gap Nav + A* hybrid',
            'Result:  25% - stuck at cone group 1',
            'Issue:   reactive approach insufficient',
        ],
        'metrics_noobs': [
            'Method:  Gap Nav + A* hybrid',
            'Result:  100% baseline',
        ],
    },
    '40_tr_nav2_hybrid': {
        'title_obs': 'Exp 40: T&R + Nav2 Single Goal - Stuck at shrub (17m / 10%)',
        'title_noobs': None,  # no no-obs data
        'color': '#17becf',
        'metrics_obs': [
            'Method:      T&R + Nav2 single goal',
            'Result:      17m before stuck',
            'Stuck at:    roadside shrub x=-79',
            'Insight:     architecture validated,',
            '             scene/cone issues',
        ],
        'metrics_noobs': None,
    },
}


def pick(dir_, prefix):
    files = sorted(glob.glob(os.path.join(dir_, 'logs', f'{prefix}*.csv')))
    # Skip the fixed-named baseline files, prefer timestamped
    ts_files = [f for f in files if any(c.isdigit() for c in os.path.basename(f)[-20:-4])]
    return ts_files[-1] if ts_files else (files[-1] if files else None)


EXTRA_PLOTS = [
    {
        'csv': '/workspace/simulation/isaac/experiments/01_gt_navigation/results/nav2_trajectory_road_1775694913.csv',
        'output': '/workspace/simulation/isaac/experiments/01_gt_navigation/results/exp01_trajectory_v2.png',
        'title': 'Exp 01: GT Navigation on Road (baseline teach-and-repeat)',
        'x_col': 'gt_x', 'y_col': 'gt_y', 'color': '#1f77b4',
        'label': 'GT navigation (baseline)',
        'metrics': [
            'Method:   GT-based teach & repeat',
            'Role:     Baseline for later experiments',
            'Route:    Road S-curve, 167m outbound',
            'Result:   Full route completion',
        ],
        'with_obstacles': False,
    },
    {
        'csv': '/workspace/simulation/isaac/experiments/34_traversability/logs/trav_road_1776089127.csv',
        'output': '/workspace/simulation/isaac/experiments/34_traversability/results/exp34_trajectory_v2.png',
        'title': 'Exp 34: Traversability Cost - Road',
        'x_col': 'gt_x', 'y_col': 'gt_y', 'color': '#17becf',
        'label': 'Traversability-based nav',
        'metrics': [
            'Method:   Traversability cost map',
            'Route:    Road',
            'Purpose:  pre-cursor to obstacle avoidance',
        ],
        'with_obstacles': False,
    },
]

for ep in EXTRA_PLOTS:
    os.makedirs(os.path.dirname(ep['output']), exist_ok=True)
    if os.path.exists(ep['csv']):
        plot_trajectory_map(
            trajectories=[{
                'csv': ep['csv'], 'label': ep['label'],
                'color': ep['color'],
                'x_col': ep['x_col'], 'y_col': ep['y_col'],
            }],
            output=ep['output'],
            title=ep['title'],
            metrics_lines=ep['metrics'],
            with_obstacles=ep.get('with_obstacles', False),
            with_waypoints=True,
        )

for exp, cfg in CONFIGS.items():
    exp_dir = f'{BASE}/{exp}'
    results = f'{exp_dir}/results'
    os.makedirs(results, exist_ok=True)

    # No obstacles
    if cfg.get('title_noobs'):
        noobs = pick(exp_dir, 'road_noobs_')
        if noobs:
            plot_trajectory_map(
                trajectories=[{
                    'csv': noobs, 'label': f'{exp.split("_", 1)[1]} (no obs)',
                    'color': cfg['color'], 'x_col': 'gt_x', 'y_col': 'gt_y',
                }],
                output=f'{results}/{exp}_trajectory_noobs_v2.png',
                title=cfg['title_noobs'],
                metrics_lines=cfg.get('metrics_noobs'),
                with_obstacles=False,
                with_waypoints=True,
            )

    # With obstacles
    obs = pick(exp_dir, 'road_obs_')
    if obs and cfg.get('title_obs'):
        plot_trajectory_map(
            trajectories=[{
                'csv': obs, 'label': f'{exp.split("_", 1)[1]} (with obs)',
                'color': cfg['color'], 'x_col': 'gt_x', 'y_col': 'gt_y',
            }],
            output=f'{results}/{exp}_trajectory_obs_v2.png',
            title=cfg['title_obs'],
            metrics_lines=cfg.get('metrics_obs'),
            with_obstacles=True,
            with_waypoints=True,
        )
