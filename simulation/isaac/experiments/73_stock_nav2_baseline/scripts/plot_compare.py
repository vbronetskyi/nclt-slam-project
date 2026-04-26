#!/usr/bin/env python3
"""Canonical-style comparison plots for exp 73 vs our custom T&R on 09_se_ne

  compare_routes.png              GT trajectories (our custom, stock Nav2) on scene map
  compare_localisation_ours.png   VIO raw + anchor-corrected + GT - our run
  compare_localisation_stock.png  VIO raw + anchor-corrected + GT - stock Nav2 run
"""
import csv, math, sys
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

ROUTE = '09_se_ne'
OUR = Path('/root/isaac_tr_datasets/09_se_ne/repeat/results/repeat_run')
STOCK = Path('/workspace/simulation/isaac/experiments/73_stock_nav2_baseline/results/run_09')
OUT_DIR = Path('/workspace/simulation/isaac/experiments/73_stock_nav2_baseline/results')
SCRATCH = OUT_DIR / 'plot_scratch'
SCRATCH.mkdir(parents=True, exist_ok=True)

#Wider frame: shows scene context (trees to the west) and stock-Nav2   
# overshoot (+-61, 49) with margin above
XLIM, YLIM = (30, 100), (-55, 60)


def split_anchor_csv(src, dst_vio, dst_anchor):
    rows = list(csv.DictReader(open(src)))
    rows = [r for r in rows if r.get('anchor_x') and r.get('anchor_y')]
    with open(dst_vio, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['x', 'y'])
        for r in rows:
            w.writerow([r['vio_x'], r['vio_y']])
    with open(dst_anchor, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['x', 'y'])
        for r in rows:
            w.writerow([r['anchor_x'], r['anchor_y']])


def mean_shift(vio_p, anc_p):
    v = list(csv.DictReader(open(vio_p)))
    a = list(csv.DictReader(open(anc_p)))
    errs = [math.hypot(float(vi['x']) - float(ai['x']),
                       float(vi['y']) - float(ai['y']))
            for vi, ai in zip(v, a)]
    return (sum(errs)/len(errs), max(errs)) if errs else (0.0, 0.0)


def main():
    our_gt = OUR / 'traj_gt.csv'
    stk_gt = STOCK / 'traj_gt.csv'

    #compare_routes.png
    plot_trajectory_map(
        trajectories=[
            {'csv': str(our_gt),  'label': 'Our custom T&R  (36/36 reached, 689 s)',
             'color': '#16a34a', 'x_col': 'x', 'y_col': 'y', 'linewidth': 2.4},
            {'csv': str(stk_gt), 'label': 'Stock Nav2 FollowWaypoints  (18/36 reached, 244 s)',
             'color': '#2563eb', 'x_col': 'x', 'y_col': 'y',
             'linewidth': 1.9},
        ],
        output=str(OUT_DIR / 'compare_routes.png'),
        title=f'{ROUTE} - our custom T&R vs stock Nav2 FollowWaypoints',
        metrics_lines=[
            'our custom     36 / 36 reached   0 skipped   dur 689 s',
            'stock Nav2 v2  18 / 36 reached  18 skipped   dur 244 s',
            '  outbound WP 0-17 OK;  at turnaround RPP could not U-turn (no',
            '  rotate-to-heading, no reversing) -> overshoot north of NE +14 m;',
            '  return WP 18-35 all NO_VALID_PATH (stale obstacle_layer inflation).',
        ],
        with_obstacles=True,
        with_waypoints=False,
        route=ROUTE,
        xlim=XLIM,
        ylim=YLIM,
        figsize=(16, 10),
    )

    #Split anchor CSVs for the localisation plots
    our_vio = SCRATCH / 'our_vio.csv'
    our_anc = SCRATCH / 'our_anchor.csv'
    stk_vio = SCRATCH / 'stock_vio.csv'
    stk_anc = SCRATCH / 'stock_anchor.csv'
    split_anchor_csv(OUR / 'anchor_matches.csv', our_vio, our_anc)
    split_anchor_csv(STOCK / 'anchor_matches.csv', stk_vio, stk_anc)
    our_mean, our_max = mean_shift(our_vio, our_anc)
    stk_mean, stk_max = mean_shift(stk_vio, stk_anc)

    # compare_localisation_ours.png
    plot_trajectory_map(
        trajectories=[
            {'csv': str(OUR / 'traj_gt.csv'),
             'label': 'GT (Isaac)', 'color': '#0f172a',
             'x_col': 'x', 'y_col': 'y', 'linewidth': 1.8},
            {'csv': str(our_vio),
             'label': 'VIO raw (at match ticks)',
             'color': '#fbbf24', 'x_col': 'x', 'y_col': 'y',
             'linewidth': 1.2},
            {'csv': str(our_anc),
             'label': f'Anchor-corrected  (mean shift {our_mean:.2f} m,  max {our_max:.2f} m)',
             'color': '#16a34a', 'x_col': 'x', 'y_col': 'y', 'linewidth': 1.6},
        ],
        output=str(OUT_DIR / 'compare_localisation_ours.png'),
        title=f'{ROUTE} - localisation (our custom T&R):  VIO raw vs anchor-corrected vs GT',
        metrics_lines=[
            f'|anchor - VIO|   mean {our_mean:.2f} m    max {our_max:.2f} m',
            f'VIO-shift grows between matches; matcher pulls it back on each',
            f'published anchor (std 0.05, shift gate 0.10 m).',
        ],
        with_obstacles=True,
        with_waypoints=False,
        route=ROUTE,
        xlim=XLIM,
        ylim=YLIM,
        figsize=(16, 10),
    )

    # compare_localisation_stock.png
    plot_trajectory_map(
        trajectories=[
            {'csv': str(STOCK / 'traj_gt.csv'),
             'label': 'GT (Isaac)', 'color': '#0f172a',
             'x_col': 'x', 'y_col': 'y', 'linewidth': 1.8},
            {'csv': str(stk_vio),
             'label': 'VIO raw (at match ticks)',
             'color': '#93c5fd', 'x_col': 'x', 'y_col': 'y',
             'linewidth': 1.2},
            {'csv': str(stk_anc),
             'label': f'Anchor-corrected  (mean shift {stk_mean:.2f} m,  max {stk_max:.2f} m)',
             'color': '#2563eb', 'x_col': 'x', 'y_col': 'y', 'linewidth': 1.6},
        ],
        output=str(OUT_DIR / 'compare_localisation_stock.png'),
        title=f'{ROUTE} - localisation (stock Nav2 run):  VIO raw vs anchor-corrected vs GT',
        metrics_lines=[
            f'|anchor - VIO|   mean {stk_mean:.2f} m    max {stk_max:.2f} m',
            f'same VIO + visual_landmark_matcher as ours; run cut short at 244 s,',
            f'so fewer match samples than our run.',
        ],
        with_obstacles=True,
        with_waypoints=False,
        route=ROUTE,
        xlim=XLIM,
        ylim=YLIM,
        figsize=(16, 10),
    )

    print('')
    print(f'routes       -> {OUT_DIR}/compare_routes.png')
    print(f'loc (ours)   -> {OUT_DIR}/compare_localisation_ours.png')
    print(f'loc (stock)  -> {OUT_DIR}/compare_localisation_stock.png')


if __name__ == '__main__':
    main()
