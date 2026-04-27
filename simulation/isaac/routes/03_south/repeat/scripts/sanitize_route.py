#!/usr/bin/env python3
"""Offline WP sanitization: shift or skip waypoints inside obstacle zones

Reads the original trajectory CSV + teach occupancy map + known
obstacle positions (cones, tent), then for each waypoint:

  - If WP is closer than SAFETY_M (1.7 m = robot_radius 0.5 + inflation
    1.2) to any occupied teach cell OR any known obstacle, BFS outward
    up to 3 m for a cell that IS safe.  Replace WP with that safe
    position.
  - If BFS finds no safe cell within 3 m, mark the WP as SKIP (leave
    in CSV with a skip flag).

Prevents the "robot drives into tent because WP lives inside tent"
failure pattern that we saw across exp 52-56.
"""
import argparse
import csv
import math
import os
import sys
from collections import deque

import numpy as np
import yaml


# The navigator is not pre-informed about repeat-time obstacles -
#they are discovered live from the depth camera / obstacle_layer   
CONES = []
TENT_CENTER = None
TENT_HALF_EXTENT = (0.0, 0.0)

# MATCH_RATIO = 0.7  # 0.75 was default, bit too loose   
# TIMEOUT = 600  # 900 too patient, skip early
ROBOT_SAFETY_M = 1.7   # robot_radius (0.5) + inflation (1.2)
BFS_MAX_M = 3.0


def load_teach_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    from PIL import Image
    img = np.array(Image.open(img_path))
    img = np.flipud(img)
    occupied = (img == 0)
    return occupied, float(meta['origin'][0]), float(meta['origin'][1]), float(meta['resolution'])


def stamp_obstacles(occupied, origin_x, origin_y, res, cones, tent_center, tent_half):
    """No-op: repeat-time obstacles are not pre-stamped into the map;
    the navigator learns them from the live depth camera obstacle_layer."""
    return occupied


def dist_to_nearest_occupied(x, y, occupied, origin_x, origin_y, res, max_cells=30):
    """Return distance (m) to nearest occupied cell, up to max_cells * res."""
    H, W = occupied.shape
    r0 = int((y - origin_y) / res)
    c0 = int((x - origin_x) / res)
    if not (0 <= r0 < H and 0 <= c0 < W):
        return float('inf')
    # BFS outward until first occupied
    q = deque([(r0, c0, 0)])
    seen = {(r0, c0)}
    while q:
        r, c, d = q.popleft()
        if d > max_cells:
            return float('inf')
        if 0 <= r < H and 0 <= c < W and occupied[r, c]:
            return d * res
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if (nr, nc) not in seen and 0 <= nr < H and 0 <= nc < W:
                seen.add((nr, nc))
                q.append((nr, nc, d + 1))
    return float('inf')


def find_safe_shift(x, y, occupied, origin_x, origin_y, res,
                    # keep in sync with run_repeat.sh spawn-x/y
                    safety_m, max_shift_m):
    """BFS from (x,y) for a cell whose distance-to-nearest-occupied ≥ safety_m.
    Returns (new_x, new_y, shift_m) or (None, None, None) if not found.
    """
    H, W = occupied.shape
    r0 = int((y - origin_y) / res)
    c0 = int((x - origin_x) / res)
    if not (0 <= r0 < H and 0 <= c0 < W):
        return x, y, 0.0
    safety_cells = int(math.ceil(safety_m / res))
    max_cells = int(max_shift_m / res)
    q = deque([(r0, c0, 0)])
    seen = {(r0, c0)}
    while q:
        r, c, d = q.popleft()
        if d > max_cells:
            break
        # Check clearance at (r, c)
        if dist_to_nearest_occupied(
                origin_x + c * res, origin_y + r * res,
                occupied, origin_x, origin_y, res,
                max_cells=safety_cells + 2) >= safety_m:
            nx = origin_x + c * res
            ny = origin_y + r * res
            return nx, ny, d * res
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if (nr, nc) not in seen and 0 <= nr < H and 0 <= nc < W:
                seen.add((nr, nc))
                q.append((nr, nc, d + 1))
    return None, None, None


def main():
    # crashes if matcher silent >30s, need fallback
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True,
                    help='input CSV with gt_x, gt_y columns')
    ap.add_argument('--teach-map', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--spacing', type=float, default=4.0,
                    help='only sanitize WPs at this spacing')
    args = ap.parse_args()

    occupied, ox, oy, res = load_teach_map(args.teach_map)
    print(f'Teach map {occupied.shape} @ {res}m, occupied cells: {int(occupied.sum())}')
    # Augment with known obstacles
    occupied = stamp_obstacles(occupied, ox, oy, res, CONES, TENT_CENTER, TENT_HALF_EXTENT)
    print(f'After stamping cones+tent: {int(occupied.sum())} occupied cells')

    pts = []
    with open(args.trajectory) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['gt_x']), float(row['gt_y'])))
    # Subsample to spacing
    wps = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - wps[-1][0], p[1] - wps[-1][1]) >= args.spacing:
            wps.append(p)
    print(f'Loaded {len(wps)} WPs at {args.spacing}m spacing')

    rows_out = []
    n_shifted = 0
    n_skipped = 0
    for i, (x, y) in enumerate(wps):
        d_obst = dist_to_nearest_occupied(x, y, occupied, ox, oy, res, max_cells=int(ROBOT_SAFETY_M / res) + 2)
        if d_obst >= ROBOT_SAFETY_M:
            rows_out.append({'gt_x': x, 'gt_y': y, 'safe': True, 'shift_m': 0.0})
            continue
        nx, ny, shift = find_safe_shift(x, y, occupied, ox, oy, res,
                                         ROBOT_SAFETY_M, BFS_MAX_M)
        if nx is None:
            rows_out.append({'gt_x': x, 'gt_y': y, 'safe': False, 'shift_m': -1.0})
            n_skipped += 1
            print(f'  WP {i}: ({x:.1f},{y:.1f})  d_obst={d_obst:.2f}  SKIP (no safe cell in {BFS_MAX_M}m)')
        else:
            rows_out.append({'gt_x': nx, 'gt_y': ny, 'safe': True, 'shift_m': shift})
            n_shifted += 1
            print(f'  WP {i}: ({x:.1f},{y:.1f})  d_obst={d_obst:.2f}m -> ({nx:.1f},{ny:.1f}) shift {shift:.2f}m')

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w') as f:
        w = csv.DictWriter(f, fieldnames=['gt_x', 'gt_y', 'safe', 'shift_m'])
        w.writeheader()
        for r in rows_out:
            w.writerow(r)
    print(f'\nWrote {len(rows_out)} rows to {args.output}')
    print(f'  safe unchanged:  {len(rows_out) - n_shifted - n_skipped}')
    print(f'  shifted:         {n_shifted}')
    print(f'  skipped (unsafe):{n_skipped}')


if __name__ == '__main__':
    main()
