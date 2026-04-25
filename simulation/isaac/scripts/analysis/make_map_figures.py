#!/usr/bin/env python3
"""thesis-grade map figures for results/final
"""
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

ROOT = Path('/workspace/simulation/isaac')
OUT = ROOT / 'results' / 'final'
OUT.mkdir(parents=True, exist_ok=True)
DATASETS = Path('/root/isaac_tr_datasets')

SCENE_JSON = Path('/tmp/gazebo_models.json')


# helpers   

def load_teach_map(route):
    """return (costgrid HxW uint8 with 0=free 254=lethal, origin_xy, res_m)"""
    p = DATASETS / route / 'teach' / 'teach_outputs' / 'teach_map.pgm'
    y = DATASETS / route / 'teach' / 'teach_outputs' / 'teach_map.yaml'
    if not p.is_file():
        return None
    import yaml
    with open(y) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(p))
    res = float(meta['resolution'])
    origin = (float(meta['origin'][0]), float(meta['origin'][1]))
    #teach_map convention: 0=occupied black, 205=unknown, 254=free
    # we only care about occupied vs free for costmap reconstruction
    occ = (img < 100).astype(np.uint8)
    return occ, origin, res, img


def read_traj_csv(path, x_col='x', y_col='y'):
    if not Path(path).is_file():
        return None, None, None
    xs, ys, ts = [], [], []
    with open(path) as f:
        rd = csv.DictReader(f)
        for row in rd:
            xc = x_col if x_col in row else ('x' if 'x' in row else 'gt_x')
            yc = y_col if y_col in row else ('y' if 'y' in row else 'gt_y')
            tc = 't' if 't' in row else ('ts' if 'ts' in row else None)
            try:
                xs.append(float(row[xc])); ys.append(float(row[yc]))
                ts.append(float(row[tc]) if tc else len(xs))
            except (ValueError, KeyError):
                continue
    return np.array(xs), np.array(ys), np.array(ts)


def add_scene_objects(ax, with_labels=False):
    if not SCENE_JSON.is_file():
        return
    try:
        scene = json.loads(SCENE_JSON.read_text())
    except Exception:
        return
    for m in scene:
        t = m.get('type', '')
        x, y = m.get('x', 0), m.get('y', 0)
        if t in ('pine', 'oak'):
            ax.add_patch(Circle((x, y), 1.3, facecolor='#1e4d1e',
                                edgecolor='#0d2d0d', linewidth=0.2,
                                alpha=0.55, zorder=4))
        elif t == 'shrub':
            ax.add_patch(Circle((x, y), 0.4, facecolor='#3a6b2e',
                                alpha=0.4, zorder=4))
        elif t == 'rock':
            ax.add_patch(Circle((x, y), 0.8, facecolor='#6b6b6b',
                                edgecolor='#3a3a3a', linewidth=0.3,
                                alpha=0.6, zorder=4))
        elif t == 'house':
            ax.add_patch(Rectangle((x - 3, y - 3), 6, 6, facecolor='#8b5a2b',
                                   edgecolor='#3a2810', linewidth=0.6,
                                   alpha=0.75, zorder=4))


# figures

def fig_10_teach_map_road():
    """teach-derived static map of route 01_road BEFORE the repeat run"""
    route = '01_road'
    loaded = load_teach_map(route)
    if loaded is None:
        print('10: no teach_map'); return
    occ, origin, res, raw = loaded
    h, w = occ.shape
    extent = (origin[0], origin[0] + w * res,
              origin[1], origin[1] + h * res)

    fig, ax = plt.subplots(figsize=(15, 6))
    ax.set_facecolor('#6b8e4e')
    # show raw teach map: occupied = dark, free = pale, unknown = grey
    disp = np.full_like(raw, 220, dtype=np.uint8)
    disp[raw < 100] = 40          # occupied
    disp[raw > 240] = 245         # free
    # ROS PGM convention: row 0 is the NORTH-most row (top of image)   
    # imshow(origin='lower') would put row 0 at the bottom of the axes,
    # which flips the map relative to world y.  flipud before imshow fixes
    # it so that occupied cells align with the teach trajectory
    ax.imshow(np.flipud(disp), origin='lower', extent=extent, cmap='gray',
              vmin=0, vmax=255, alpha=0.9, zorder=2)

    add_scene_objects(ax)

    teach = DATASETS / route / 'teach' / 'teach_outputs' / 'traj_gt_world.csv'
    tx, ty, _ = read_traj_csv(teach, 'x', 'y')
    if tx is not None:
        ax.plot(tx, ty, color='#111827', lw=1.6, ls='--',
                label='teach GT trajectory', zorder=7)
        step = max(1, len(tx) // 40)
        ax.plot(tx[::step], ty[::step], 'o', ms=3.5, color='#fbbf24',
                markeredgecolor='#7a5a1e', lw=0, label='teach WPs (+-4 m)',
                zorder=8)

    # spawn + turnaround markers
    ax.plot(tx[0], ty[0], marker='*', ms=18, color='#16a34a',
            markeredgecolor='#000', lw=0, label='spawn', zorder=9)
    i_apex = int(np.argmax(tx))
    ax.plot(tx[i_apex], ty[i_apex], marker='X', ms=15, color='#dc2626',
            markeredgecolor='#000', lw=0, label='turnaround', zorder=9)

    ax.set_xlim(-107, 82); ax.set_ylim(-16, 11)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('teach-derived static occupancy for 01_road  '
                 '(before repeat run - no obstacles yet)',
                 fontsize=11, fontweight='bold')
    ax.grid(alpha=0.2)
    ax.legend(loc='upper right', fontsize=9, ncol=2, framealpha=0.92)
    plt.tight_layout()
    plt.savefig(OUT / '10_teach_map_road.png', dpi=140)
    plt.close()
    print('10_teach_map_road.png')


def fig_10b_teach_map_south():
    """teach-derived static map of route 03_south (south-forest loop).
    user asked for the atlas built during driving the 3rd route, rendered
    as its own figure."""
    route = '03_south'
    loaded = load_teach_map(route)
    if loaded is None:
        print('10b: no teach_map'); return
    occ, origin, res, raw = loaded
    h, w = occ.shape
    extent = (origin[0], origin[0] + w * res,
              origin[1], origin[1] + h * res)

    fig, ax = plt.subplots(figsize=(15, 8))
    ax.set_facecolor('#6b8e4e')
    disp = np.full_like(raw, 220, dtype=np.uint8)
    disp[raw < 100] = 40          # occupied
    disp[raw > 240] = 245         # free
    # ROS PGM convention: row 0 is the NORTH-most row (top of image)
    # imshow(origin='lower') would put row 0 at the bottom of the axes,
    # which flips the map relative to world y.  flipud before imshow fixes
    #it so that occupied cells align with the teach trajectory
    ax.imshow(np.flipud(disp), origin='lower', extent=extent, cmap='gray',
              vmin=0, vmax=255, alpha=0.9, zorder=2)

    add_scene_objects(ax)

    # 03_south teach used the earlier pipeline and has no traj_gt_world.csv
    # - fall back to vio_pose_dense.csv's gt_x/gt_y (the teach run was driven
    # in --use-gt so vio == gt here anyway)
    teach = DATASETS / route / 'teach' / 'teach_outputs' / 'traj_gt_world.csv'
    if not teach.is_file():
        teach = DATASETS / route / 'teach' / 'teach_outputs' / 'vio_pose_dense.csv'
    tx, ty, _ = read_traj_csv(teach, 'gt_x', 'gt_y')
    if tx is None or not len(tx):
        tx, ty, _ = read_traj_csv(teach, 'x', 'y')
    if tx is not None and len(tx):
        ax.plot(tx, ty, color='#111827', lw=1.6, ls='--',
                label='teach GT trajectory', zorder=7)
        step = max(1, len(tx) // 50)
        ax.plot(tx[::step], ty[::step], 'o', ms=3.5, color='#fbbf24',
                markeredgecolor='#7a5a1e', lw=0, label='teach WPs (+-4 m)',
                zorder=8)
        ax.plot(tx[0], ty[0], marker='*', ms=18, color='#16a34a',
                markeredgecolor='#000', lw=0, label='spawn', zorder=9)
        i_apex = int(np.argmax(np.abs(tx - tx[0]) + np.abs(ty - ty[0])))
        ax.plot(tx[i_apex], ty[i_apex], marker='X', ms=15, color='#dc2626',
                markeredgecolor='#000', lw=0, label='turnaround', zorder=9)

    ax.set_xlim(origin[0], origin[0] + w * res)
    ax.set_ylim(origin[1], origin[1] + h * res)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    total_occupied = int((raw < 100).sum())
    total_free = int((raw > 240).sum())
    ax.set_title(
        'teach-derived static atlas for 03_south  '
        f'(depth-mapped occupancy, 0.1 m/px, {w} x {h} cells)  |  '
        f'{total_occupied:,} occupied · {total_free:,} free',
        fontsize=11, fontweight='bold')
    ax.grid(alpha=0.2)
    ax.legend(loc='upper right', fontsize=9, ncol=2, framealpha=0.92)
    plt.tight_layout()
    plt.savefig(OUT / '10b_teach_map_south.png', dpi=140)
    plt.close()
    print('10b_teach_map_south.png')


def fig_10e_teach_map_landmarks_south():
    """teach atlas for 03_south with recorded visual landmarks overlaid.
    landmarks are ORB descriptors + back-projected 3D keypoints captured
    every +-2 m of teach displacement, used at repeat time by the visual
    landmark matcher for anchor correction"""
    import pickle
    route = '03_south'
    loaded = load_teach_map(route)
    if loaded is None:
        print('10e: no teach_map'); return
    occ, origin, res, raw = loaded
    h, w = occ.shape
    extent = (origin[0], origin[0] + w * res,
              origin[1], origin[1] + h * res)

    pkl = DATASETS / route / 'teach' / 'teach_outputs' / 'landmarks.pkl'
    if not pkl.is_file():
        pkl = DATASETS / route / 'teach' / 'teach_outputs' / 'south_landmarks.pkl'
    if not pkl.is_file():
        print('10e: no landmarks.pkl'); return
    with open(pkl, 'rb') as f:
        data = pickle.load(f)
    lms = data['landmarks']
    lm_x, lm_y, lm_ts, lm_n = [], [], [], []
    for lm in lms:
        p = lm['pose']
        lm_x.append(float(p[0])); lm_y.append(float(p[1]))
        lm_ts.append(float(lm.get('ts', 0)))
        lm_n.append(int(lm.get('n_features', 0)))
    lm_x = np.array(lm_x); lm_y = np.array(lm_y)
    lm_ts = np.array(lm_ts); lm_n = np.array(lm_n)

    fig, ax = plt.subplots(figsize=(15, 8.4))
    ax.set_facecolor('#6b8e4e')
    disp = np.full_like(raw, 220, dtype=np.uint8)
    disp[raw < 100] = 40
    disp[raw > 240] = 245
    ax.imshow(np.flipud(disp), origin='lower', extent=extent, cmap='gray',
              vmin=0, vmax=255, alpha=0.85, zorder=2)
    add_scene_objects(ax)

    teach = DATASETS / route / 'teach' / 'teach_outputs' / 'vio_pose_dense.csv'
    if teach.is_file():
        ttx, tty, _ = read_traj_csv(teach, 'gt_x', 'gt_y')
        if ttx is not None:
            ax.plot(ttx, tty, color='#111827', lw=1.3, ls='--', alpha=0.55,
                    label='teach GT trajectory', zorder=5)

    order = np.argsort(lm_ts)
    sc = ax.scatter(lm_x[order], lm_y[order],
                    c=np.arange(len(order)), cmap='plasma',
                    s=np.clip(lm_n[order] * 0.6, 8, 50),
                    edgecolors='#000', linewidths=0.5,
                    alpha=0.92, zorder=8,
                    label=f'{len(lms)} teach landmarks  (size ∝ n_features)')

    ax.plot(lm_x[order[0]], lm_y[order[0]], marker='*', ms=18,
            color='#16a34a', markeredgecolor='#000', lw=0,
            label='first landmark (spawn)', zorder=9)
    ax.plot(lm_x[order[-1]], lm_y[order[-1]], marker='P', ms=15,
            color='#dc2626', markeredgecolor='#000', lw=0,
            label='last landmark (near return)', zorder=9)

    ax.set_xlim(origin[0], origin[0] + w * res)
    ax.set_ylim(origin[1], origin[1] + h * res)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    spacing = 0.0
    if len(lm_x) > 1:
        seg = np.hypot(np.diff(lm_x[order]), np.diff(lm_y[order]))
        spacing = float(np.median(seg))
    ax.set_title(
        f'03_south teach atlas + {len(lms)} ORB landmarks  '
        f'(spacing {spacing:.1f} m, {lm_n.mean():.0f} features avg)',
        fontsize=11, fontweight='bold', pad=8)
    ax.grid(alpha=0.2)
    # legend below the axes - leaves both title and atlas clear
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08),
              fontsize=9, ncol=4, framealpha=0.95, facecolor='white',
              edgecolor='#444')
    # compact colorbar on the right, short enough to stay below the title
    cax = fig.add_axes([0.91, 0.25, 0.012, 0.45])
    cbar = plt.colorbar(sc, cax=cax, orientation='vertical')
    cbar.set_label('landmark record order  (early → late)', fontsize=9)
    plt.subplots_adjust(left=0.05, right=0.9, top=0.92, bottom=0.18)
    plt.savefig(OUT / '10e_teach_map_landmarks_south.png', dpi=140)
    plt.close()
    print(f'10e_teach_map_landmarks_south.png  ({len(lms)} landmarks, '
          f'median spacing {spacing:.1f} m)')


def main():
    fig_10_teach_map_road()
    fig_10b_teach_map_south()
    fig_10e_teach_map_landmarks_south()
    print('done')


if __name__ == '__main__':
    main()
