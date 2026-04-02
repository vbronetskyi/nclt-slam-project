#!/usr/bin/env python3
"""Analyze exp 55 repeat run 3: trajectory_map + err-vs-dist + regime timeline."""
import os
import sys
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

R = '/workspace/simulation/isaac/experiments/56_projection_cap_1m/results/repeat_run'


def parse_traj():
    rows = []
    with open(f'{R}/trajectory.csv') as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) if k not in ('regime',) else v
                         for k, v in row.items()})
    return rows


def main():
    rows = parse_traj()
    gx = np.array([r['gt_x'] for r in rows])
    gy = np.array([r['gt_y'] for r in rows])
    err = np.array([r['err'] for r in rows])
    enc = np.array([r['enc_err'] for r in rows])
    gt_dist = float(np.hypot(np.diff(gx), np.diff(gy)).sum())
    n_pub = sum(1 for l in open(f'{R}/anchor_matches.csv').readlines()[1:] if 'published' in l)
    n_att = sum(1 for _ in open(f'{R}/anchor_matches.csv')) - 1
    n_reach = sum(1 for l in open(f'{R}/goals.log') if 'REACHED' in l)
    n_timeout = sum(1 for l in open(f'{R}/goals.log') if 'TIMEOUT' in l)
    n_skip = sum(1 for l in open(f'{R}/goals.log') if 'SKIP' in l)
    n_wedge = sum(1 for l in open(f'{R}/pp_follower.log') if 'WEDGE #' in l) if os.path.exists(f'{R}/pp_follower.log') else 0

    metrics = [
        f"Distance driven:   {gt_dist:.0f} m  (max x={gx.max():.0f}, end x={gx[-1]:.0f})",
        f"WPs reached:       {n_reach}/94  (timeout {n_timeout}, skipped {n_skip})",
        f"Wedge recoveries:  {n_wedge}",
        f"Anchor matches:    {n_pub}/{n_att} = {100*n_pub/n_att:.1f}%",
        f"SLAM err mean/med/max: {err.mean():.2f} / {np.median(err):.2f} / {err.max():.2f} m",
        f"Encoder err max:   {enc.max():.2f} m",
        f"Final drift:       {err[-1]:.2f} m at {rows[-1]['dist_m']} m driven",
    ]
    for m in metrics:
        print("  " + m)

    plot_trajectory_map(
        trajectories=[
            {'csv': f'{R}/trajectory.csv', 'label': f'Repeat GT - {n_reach}/94 WPs, {gt_dist:.0f} m',
             'color': '#1f77b4', 'x_col': 'gt_x', 'y_col': 'gt_y'},
            {'csv': f'{R}/trajectory.csv', 'label': 'SLAM/encoder fused nav',
             'color': '#ff6600', 'x_col': 'nav_x', 'y_col': 'nav_y'},
        ],
        output=f'{R}/trajectory_map.png',
        title='Exp 56 run 2 - projection cap + (ground filter reverted)',
        metrics_lines=metrics,
        with_obstacles=True, with_waypoints=True, route='south',
    )

    dist = np.array([r['dist_m'] for r in rows])
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True,
                             gridspec_kw={'height_ratios': [3, 1]})
    axes[0].plot(dist, err, '-', color='tab:red', label='SLAM/nav err (vs GT)')
    axes[0].plot(dist, enc, '-', color='tab:green', label='Encoder-only err', alpha=0.7)
    axes[0].set_ylabel('Position error (m)')
    axes[0].legend(); axes[0].grid(alpha=0.3)
    axes[0].set_title('Localization error vs distance driven')
    regimes = [r['regime'] for r in rows]
    reg_color = {'strong': 'green', 'ok': 'orange', 'no_anchor': 'lightgray'}
    for i, r in enumerate(regimes[:-1]):
        axes[1].axvspan(dist[i], dist[i+1], color=reg_color.get(r, 'gray'),
                        alpha=0.7)
    axes[1].set_xlabel('Distance driven (m)')
    axes[1].set_ylabel('Anchor\nregime')
    axes[1].set_yticks([])
    from matplotlib.patches import Patch
    axes[1].legend(handles=[Patch(color=c, label=n) for n, c in reg_color.items()],
                   loc='upper right', ncol=3)
    plt.tight_layout()
    plt.savefig(f'{R}/err_and_regime.png', dpi=120)
    plt.close()
    print(f"\n  trajectory_map.png + err_and_regime.png saved to {R}")


if __name__ == '__main__':
    main()
