#!/usr/bin/env python3
"""thesis Chapter-5 Phase-2 figures.  writes PNGs under
results/final/phase2/.  consolidates aggregate success / metrics heatmap /
anchor-frequency plots that complement the existing 14/15/16 trajectory
+ drift-curves figures.

run:  python3 scripts/analysis/make_phase2_plots.py
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, ListedColormap

ROOT = Path('/workspace/simulation/isaac')
OUT = ROOT / 'results' / 'final' / 'phase2'
OUT.mkdir(parents=True, exist_ok=True)
DATASETS = Path('/root/isaac_tr_datasets')

ROUTES_9 = ['01_road', '02_north_forest', '03_south',
            '04_nw_se', '05_ne_sw', '06_nw_ne',
            '07_se_sw', '08_nw_sw', '09_se_ne']

ROUTES_15 = ROUTES_9 + ['10_nmid_smid', '11_nw_mid', '12_ne_mid',
                        '13_cross_nws', '14_se_mid', '15_wmid_smid']

LABELS = {
    '01_road': 'road',         '02_north_forest': 'N-forest',
    '03_south': 'south',       '04_nw_se': 'NW-SE',
    '05_ne_sw': 'NE-SW',       '06_nw_ne': 'NW-NE',
    '07_se_sw': 'SE-SW',       '08_nw_sw': 'NW-SW',
    '09_se_ne': 'SE-NE',
    '10_nmid_smid': 'Nmid-Smid', '11_nw_mid': 'NW-mid',
    '12_ne_mid': 'NE-mid',       '13_cross_nws': 'cross-NWS',
    '14_se_mid': 'SE-mid',       '15_wmid_smid': 'Wmid-Smid',
}


# routes 10-15 metrics aren't in metrics.json yet (compute_metrics still
# uses the 9-route corpus); pulled from routes/README.md table
EXTRA_15 = {
    '10_nmid_smid': {'final_d': 4.2, 'return_d': 4.8, 'drift_mean': 3.0,
                     'drift_p95': 3.8, 'drift_max': 3.9, 'cov_pct': 82.5},
    '11_nw_mid':    {'final_d': 3.1, 'return_d': 5.2, 'drift_mean': 2.0,
                     'drift_p95': 2.8, 'drift_max': 2.8, 'cov_pct': 79.5},
    '12_ne_mid':    {'final_d': 1.1, 'return_d': 11.8, 'drift_mean': 5.2,
                     'drift_p95': 7.3, 'drift_max': 7.7, 'cov_pct': 82.6},
    '13_cross_nws': {'final_d': 2.6, 'return_d': 28.7, 'drift_mean': 18.8,
                     'drift_p95': 24.1, 'drift_max': 25.3, 'cov_pct': 60.5},
    '14_se_mid':    {'final_d': 3.7, 'return_d': 2.7, 'drift_mean': 2.6,
                     'drift_p95': 5.1, 'drift_max': 5.1, 'cov_pct': 28.2},
    '15_wmid_smid': {'final_d': 4.8, 'return_d': 6.5, 'drift_mean': 7.2,
                     'drift_p95': 11.5, 'drift_max': 11.8, 'cov_pct': 50.0},
}


def load_metrics():
    with open(ROOT / 'routes' / '_common' / 'metrics.json') as f:
        m = json.load(f)
    #graft 10-15 into 'our custom' slot since compute_metrics hasn't
    # extended yet   
    for r, vals in EXTRA_15.items():
        m.setdefault(r, {})['our custom'] = vals
    return m


# 17. aggregate success heatmap (9 routes x 3 stacks x reach + return)

def fig_17_success_heatmap(metrics):
    methods = [('our custom', 'our custom T&R'),
               ('exp 76 RGB-D', 'RGB-D only (no IMU)'),
               ('exp 74 stock', 'stock Nav2 (no matcher)')]
    metric_kinds = [('reach', 'final_d', 'reach to apex'),
                    ('return', 'return_d', 'return to spawn')]

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.8),
                              gridspec_kw={'wspace': 0.18})
    # ROVER-style green/yellow/red gradient (matches Phase-1 ATE heatmap)
    cmap = LinearSegmentedColormap.from_list('gyr',
        ['#2ca02c', '#ffdd44', '#d62728'])
    cmap.set_bad(color='#d0d0d0')

    for ax, (kind, field, title) in zip(axes, metric_kinds):
        grid = np.full((len(ROUTES_9), len(methods)), np.nan)
        annot = np.empty((len(ROUTES_9), len(methods)), dtype=object)
        for i, r in enumerate(ROUTES_9):
            for j, (mkey, _) in enumerate(methods):
                d = metrics.get(r, {}).get(mkey, {})
                v = d.get(field, None)
                if v is None:
                    annot[i, j] = 'n/a'; continue
                grid[i, j] = v
                annot[i, j] = f'{v:.1f} m'
        masked = np.ma.masked_invalid(grid)
        im = ax.imshow(masked, cmap=cmap, vmin=0, vmax=30, aspect='auto')
        import matplotlib.patheffects as pe
        for i in range(len(ROUTES_9)):
            for j in range(len(methods)):
                t = annot[i, j]
                if t == 'n/a': continue
                ax.text(j, i, t, ha='center', va='center',
                        fontsize=10.5, color='black', fontweight='bold')
        ax.set_yticks(range(len(ROUTES_9)))
        ax.set_yticklabels([LABELS[r] for r in ROUTES_9])
        ax.set_xticks(range(len(methods)))
        ax.set_xticklabels([m[1] for m in methods], rotation=15, ha='right')
        ax.set_title(f'{title} error  (m)', fontsize=11.5, fontweight='bold')
    cax = fig.add_axes([0.92, 0.18, 0.012, 0.65])
    cbar = plt.colorbar(im, cax=cax)
    cbar.set_label('endpoint error [m]  (clipped at 30 m)', fontsize=9)
    fig.suptitle(
        'Phase-2 endpoint success heatmap  (9 corner routes × 3 stacks)',
        fontsize=12, fontweight='bold', y=1.0)
    plt.savefig(OUT / '17_success_heatmap_9x3.png', dpi=140, bbox_inches='tight')
    plt.close()
    print('17_success_heatmap_9x3.png')


# 18. our pipeline 15-route metrics heatmap

def fig_18_metrics_heatmap_15(metrics):
    cols = [
        ('reach (m)',     'final_d',    True),    # lower better
        ('return (m)',    'return_d',   True),
        ('drift mean',    'drift_mean', True),
        ('drift max',     'drift_max',  True),
        ('coverage %',    'cov_pct',    False),   # higher better
    ]
    M = np.zeros((len(ROUTES_15), len(cols)), dtype=float)
    A = np.empty((len(ROUTES_15), len(cols)), dtype=object)
    for i, r in enumerate(ROUTES_15):
        d = metrics.get(r, {}).get('our custom', {})
        for j, (_, field, _) in enumerate(cols):
            v = d.get(field, np.nan)
            M[i, j] = v if v is not None else np.nan
            A[i, j] = '' if v is None else (
                f'{v:.0f} %' if field == 'cov_pct' else f'{v:.1f}')
    fig, ax = plt.subplots(figsize=(11, 9))
    # per-column scale (each metric different range/units), but ROVER-style
    # green/yellow/red colormap so the visual language matches Phase-1
    norm = np.zeros_like(M, dtype=float)
    for j, (_, field, lower_better) in enumerate(cols):
        col = M[:, j]
        finite = np.isfinite(col)
        lo, hi = col[finite].min(), col[finite].max()
        if hi > lo:
            n = (col - lo) / (hi - lo)
        else:
            n = np.zeros_like(col)
        norm[:, j] = n if lower_better else 1 - n
    cmap = LinearSegmentedColormap.from_list(
        'gyr', ['#2ca02c', '#ffdd44', '#d62728'])
    ax.imshow(norm, cmap=cmap, vmin=0, vmax=1, aspect='auto')
    import matplotlib.patheffects as pe
    for i in range(len(ROUTES_15)):
        for j in range(len(cols)):
            t = A[i, j]
            if not t: continue
            ax.text(j, i, t, ha='center', va='center', fontsize=10,
                    color='black', fontweight='bold')
    ax.set_yticks(range(len(ROUTES_15)))
    ax.set_yticklabels([LABELS[r] for r in ROUTES_15])
    ax.set_xticks(range(len(cols)))
    ax.set_xticklabels([c[0] for c in cols], rotation=15, ha='right')
    ax.set_title(
        'Phase-2 our-pipeline metrics across 15 routes  '
        '(green = best · amber = mid · red = worst, per-column-normalised)',
        fontsize=11, fontweight='bold')
    plt.tight_layout()
    plt.savefig(OUT / '18_metrics_heatmap_15.png', dpi=140)
    plt.close()
    print('18_metrics_heatmap_15.png')


def main():
    metrics = load_metrics()
    fig_17_success_heatmap(metrics)
    fig_18_metrics_heatmap_15(metrics)
    print('done')


if __name__ == '__main__':
    main()
