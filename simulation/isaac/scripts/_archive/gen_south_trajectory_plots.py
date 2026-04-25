#!/usr/bin/env python3
"""Generate v2 trajectory plots for south/north route experiments (30-33)
"""
import glob
import os
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

BASE = '/workspace/simulation/isaac/experiments'


def pick_newest(dir_, prefix):
    files = sorted(glob.glob(os.path.join(dir_, 'logs', f'{prefix}*.csv')))
    return files[-1] if files else None


# Exp 30: teach_and_repeat (south route)
exp = '30_teach_and_repeat'
results = f'{BASE}/{exp}/results'
os.makedirs(results, exist_ok=True)
for prefix, label, title, color, metrics in [
    ('teach_repeat_south_outbound_', 'GT controller (99%)',
     'Exp 30: Teach-and-Repeat GT Controller - South Route (99%)',
     '#2ca02c',
     ['Method:   Pure pursuit + GT',
      'Route:    South forest, 196m',
      'Result:   99% completion',
      'Baseline for T&R experiments']),
    ('odom_imu_south_', 'Encoder+IMU dead reckoning',
     'Exp 30: Encoder+IMU Only - South Route',
     '#ff7f0e',
     ['Method:   Encoder + IMU (no vision)',
      'Route:    South forest',
      'Result:   +-25m before drift failure',
      'Shows: dead-reckoning alone insufficient']),
    ('teach_then_repeat_south_', 'Full T&R pipeline',
     'Exp 30: Full Teach-then-Repeat - South Route',
     '#1f77b4',
     ['Method:   Visual anchor matching',
      'Route:    South forest',
      'Result:   72m progress (mixed)']),
]:
    csv = pick_newest(f'{BASE}/{exp}', prefix)
    if csv:
        plot_trajectory_map(
            trajectories=[{'csv': csv, 'label': label, 'color': color,
                           'x_col': 'gt_x', 'y_col': 'gt_y'}],
            output=f'{results}/{exp}_{prefix.rstrip('_')}_v2.png',
            title=title, metrics_lines=metrics,
            route='south', with_obstacles=False, with_waypoints=False,
        )

# Exp 31: gap_navigator
exp = '31_gap_navigator'
results = f'{BASE}/{exp}/results'
os.makedirs(results, exist_ok=True)
csv = pick_newest(f'{BASE}/{exp}', 'gap_nav_south_')
if csv:
    plot_trajectory_map(
        trajectories=[{'csv': csv, 'label': 'Gap Navigator', 'color': '#9467bd',
                       'x_col': 'gt_x', 'y_col': 'gt_y'}],
        output=f'{results}/{exp}_trajectory_v2.png',
        title='Exp 31: Gap Navigator - South Route',
        metrics_lines=[
            'Method:   Reactive gap navigation',
            'Route:    South forest',
            'Purpose:  Handle forest obstacles',
        ],
        route='south', with_obstacles=False, with_waypoints=False,
    )

# Exp 32: robust_anchoring
exp = '32_robust_anchoring'
results = f'{BASE}/{exp}/results'
os.makedirs(results, exist_ok=True)
csv = pick_newest(f'{BASE}/{exp}', 'robust_')
if csv:
    plot_trajectory_map(
        trajectories=[{'csv': csv, 'label': 'Robust anchoring', 'color': '#8c564b',
                       'x_col': 'gt_x', 'y_col': 'gt_y'}],
        output=f'{results}/{exp}_trajectory_v2.png',
        title='Exp 32: Robust Anchoring - South Route',
        metrics_lines=[
            'Method:   Robust visual anchor matching',
            'Route:    South forest',
            'Purpose:  SLAM stale detection',
        ],
        route='south', with_obstacles=False, with_waypoints=False,
    )

#Exp 34 south (in addition to road done earlier)
exp = '34_traversability'
results = f'{BASE}/{exp}/results'
csv = pick_newest(f'{BASE}/{exp}', 'trav_south_')
if csv:
    plot_trajectory_map(
        trajectories=[{'csv': csv, 'label': 'Traversability', 'color': '#17becf',
                       'x_col': 'gt_x', 'y_col': 'gt_y'}],
        output=f'{results}/{exp}_trajectory_south_v2.png',
        title='Exp 34: Traversability - South Route',
        metrics_lines=[
            'Method:   Traversability cost',
            'Route:    South forest',
        ],
        route='south', with_obstacles=False, with_waypoints=False,
    )
