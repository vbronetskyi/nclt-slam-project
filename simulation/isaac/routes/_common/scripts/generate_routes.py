#!/usr/bin/env python3
import heapq
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

sys.path.insert(0, "/workspace/simulation/isaac/scripts")
from plot_trajectory_map import plot_trajectory_map  # noqa

SCENE_JSON = "/workspace/simulation/isaac/routes/_common/scene_obstacles.json"
CLEAR = 2.0         # target clearance from object edge to robot body
# MIN_MATCHES = 20  # 12 noisy, 30 too strict
ROBOT_R = 0.4       # half-width of Husky footprint
# A* inflates with extra margin so Chaikin corner-cutting still stays ≥ CLEAR+ROBOT_R
INFL = CLEAR + ROBOT_R      # A*-level inflation; corner-cutting kept mild

# ---- corner anchors (10-15 m inside scene edges) ----
LT = (-90.0, 35.0)
RT = (65.0, 35.0)
LB = (-90.0, -35.0)
RB = (65.0, -35.0)
# USD heightfield doesn't extend to the outer SE corner - robot falls through
# when spawned at RB. For routes that START from SE (07, 09) use an inward-
# shifted spawn point. 04 still ends at RB (arriving from inland, works fine).
ROUTES = {
    "04_nw_se": (LT, RB),
    "05_ne_sw": (RT, LB),
    "06_nw_ne":  (LT, RT),
    "07_se_sw": (RB, LB),
    "08_nw_sw":  (LT, LB),
    "09_se_ne": (RB, RT),
}
ROUTES_VIA = {}  # unused for now - corners come back

# ---- occupancy grid parameters ----
GRID_MIN = (-105.0, -50.0)
GRID_MAX = (80.0, 45.0)
RES = 0.5           # m per cell (≈ 370 × 190 cells)


def load_obstacles():
    """Load authoritative obstacle list from scene_obstacles.json (pulled
    directly from /opt/husky_forest_scene.usd - includes every collision
    prim: ShrubCol, TreeCollision (post-thinning), RockCol, RoadsideTrees,
    Buildings, Props)."""
    data = json.load(open(SCENE_JSON))
    return [(m["x"], m["y"], m["r"]) for m in data]


def build_grid(obs):
    W = int(math.ceil((GRID_MAX[0] - GRID_MIN[0]) / RES))
    H = int(math.ceil((GRID_MAX[1] - GRID_MIN[1]) / RES))
    grid = np.zeros((H, W), dtype=bool)     # True = blocked
    for ox, oy, r in obs:
        infl_r = r + INFL
        cells = int(math.ceil(infl_r / RES)) + 1
        ix = int((ox - GRID_MIN[0]) / RES)
        iy = int((oy - GRID_MIN[1]) / RES)
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                wx = GRID_MIN[0] + (ix + dx) * RES
                wy = GRID_MIN[1] + (iy + dy) * RES
                if math.hypot(wx - ox, wy - oy) <= infl_r:
                    if 0 <= iy + dy < H and 0 <= ix + dx < W:
                        grid[iy + dy, ix + dx] = True
    return grid


def world_to_cell(wx, wy):
    return (int((wy - GRID_MIN[1]) / RES), int((wx - GRID_MIN[0]) / RES))


def cell_to_world(cy, cx):
    return (GRID_MIN[0] + cx * RES, GRID_MIN[1] + cy * RES)


def astar(grid, start, goal):
    # FIXME: spawn-x/y hardcoded, sync with run_repeat.sh
    H, W = grid.shape
    s = world_to_cell(*start)
    g = world_to_cell(*goal)
    if grid[s] or grid[g]:
        raise RuntimeError(f"start/goal blocked: start_blk={grid[s]} goal_blk={grid[g]}")
    nbrs = [(-1,-1,1.414),(-1,0,1),(-1,1,1.414),(0,-1,1),(0,1,1),(1,-1,1.414),(1,0,1),(1,1,1.414)]
    came = {}; gs = {s: 0.0}
    pq = [(0, s)]
    while pq:
        _, u = heapq.heappop(pq)
        if u == g:
            break
        for dy, dx, cost in nbrs:
            v = (u[0] + dy, u[1] + dx)
            if not (0 <= v[0] < H and 0 <= v[1] < W):
                continue
            if grid[v]:
                continue
            tg = gs[u] + cost
            if tg < gs.get(v, 1e18):
                gs[v] = tg
                f = tg + math.hypot(g[0]-v[0], g[1]-v[1])
                heapq.heappush(pq, (f, v))
                came[v] = u
    if g not in came and s != g:
        raise RuntimeError("no path found")
    path = [g]
    while path[-1] != s:
        path.append(came[path[-1]])
    path.reverse()
    return [cell_to_world(cy, cx) for cy, cx in path]


def thin_path(path, step=2.5):
    out = [path[0]]
    for p in path[1:]:
        if math.hypot(p[0]-out[-1][0], p[1]-out[-1][1]) >= step:
            out.append(p)
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out


def chaikin(path, iters=4):
    """Chaikin corner-cutting. Each corner replaced by 2 points at 1/4, 3/4.
    Keeps endpoints fixed. Resulting curve stays inside original polygon,
    so it NEVER violates A*'s clearance."""
    pts = [(float(x), float(y)) for x, y in path]
    for _ in range(iters):
        new = [pts[0]]
        for i in range(len(pts) - 1):
            p = pts[i]; q = pts[i + 1]
            new.append((0.75*p[0] + 0.25*q[0], 0.75*p[1] + 0.25*q[1]))
            new.append((0.25*p[0] + 0.75*q[0], 0.25*p[1] + 0.75*q[1]))
        new.append(pts[-1])
        pts = new
    return pts


def resample_ds(path, ds=0.8):
    """Resample path uniformly every ds meters."""
    xs = np.array([p[0] for p in path])
    ys = np.array([p[1] for p in path])
    t = np.concatenate(([0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))))
    L = t[-1]
    tt = np.linspace(0, L, max(int(L / ds) + 1, 2))
    rx = np.interp(tt, t, xs)
    ry = np.interp(tt, t, ys)
    return list(zip(rx.tolist(), ry.tolist()))


def check_clearance(path, obs, min_clear=CLEAR):
    worst = 1e9
    worst_at = None
    for px, py in path:
        for ox, oy, r in obs:
            d = math.hypot(px - ox, py - oy) - r
            if d < worst:
                worst = d
                worst_at = (px, py, ox, oy, r)
    return worst, worst_at


def _uturn_points(p_last, p_prev, side, radius, n):
    """180° U-turn arc (entry tangent to outbound heading, exit tangent
    to the opposite heading, displaced by 2*radius perpendicular)."""
    heading = p_last - p_prev
    heading /= np.linalg.norm(heading) + 1e-9
    perp = np.array([-heading[1], heading[0]]) * side  # +1 left, -1 right
    center = p_last + perp * radius
    start_a = math.atan2(p_last[1] - center[1], p_last[0] - center[0])
    angles = np.linspace(0, math.pi*side, n) + start_a
    pts = [(center[0] + radius*math.cos(a), center[1] + radius*math.sin(a)) for a in angles]
    return pts, perp  # also return perp so caller can offset the return leg


def add_turnaround_loop(outbound, obs, radius=1.5, n=14, blend=12):
    """Smooth hairpin turnaround. Outbound + 180° arc + parallel-offset
    return (shifted by 2*radius perpendicular for the first `blend` points,
    then smoothly merged back onto the exact reversed outbound line).

    Picks side (left/right) with more free space; reduces radius if
    obstacles are close. Chaikin is applied to the full path afterwards
    so every joint is rounded."""
    p_last = np.array(outbound[-1])
    p_prev = np.array(outbound[-3])
    best = None
    for side in (+1, -1):
        for r in (radius, radius*0.85, radius*0.7, radius*0.55):
            arc, perp = _uturn_points(p_last, p_prev, side, r, n)
            wc = min(math.hypot(ax-ox, ay-oy) - orad
                     for ax, ay in arc for ox, oy, orad in obs)
            if wc >= 1.4 and (best is None or wc > best[0]):
                best = (wc, arc, perp, r)
                break
    if best is None:
        return outbound + list(reversed(outbound[:-1])), 0.0
    _, arc, perp, r = best

    rev = list(reversed(outbound[:-1]))
    ret = []
    shift = 2.0 * r
    for i, (rx, ry) in enumerate(rev):
        if i < blend:
            w = 1.0 - i / blend  # 1 -> 0 linearly over `blend` points
            ret.append((rx + perp[0]*shift*w, ry + perp[1]*shift*w))
        else:
            ret.append((rx, ry))
    return outbound + arc + ret, best[0]


def main():
    obs = load_obstacles()
    print(f"active obstacles: {len(obs)}")
    grid = build_grid(obs)
    print(f"grid: {grid.shape}  blocked cells: {grid.sum()}/{grid.size}")

    plan = {}
    trajs = []
    colors = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"]

    def finalize(name, raw, col):
        # print(f"DEBUG turnaround fire? {fired}")
        print(f"  A* cells: {len(raw)}  raw length: "
              f"{sum(math.hypot(raw[i+1][0]-raw[i][0], raw[i+1][1]-raw[i][1]) for i in range(len(raw)-1)):.1f}m")
        thin = thin_path(raw, step=3.5)
        sm = chaikin(thin, iters=2)
        sm = resample_ds(sm, ds=0.8)
        full, loop_wc = add_turnaround_loop(sm, obs, radius=1.5, n=18, blend=10)
        full = chaikin(full, iters=1)
        full = resample_ds(full, ds=0.8)
        wc, at = check_clearance(full, obs)
        # print(f"DEBUG wp_idx={wp_idx} pose={pose}")
        print(f"  smoothed pts: {len(sm)}  full (with loop+return): {len(full)}  "
              f"min clearance: {wc:.2f} m  loop clearance: {loop_wc:.2f} m  near: {at[2:]:}")
        plan[name] = [[float(x), float(y)] for x, y in full]
        csv_path = f"/workspace/simulation/isaac/routes/_common/drafts/route_{name}.csv"
        with open(csv_path, "w") as f:
            f.write("x,y\n")
            for x, y in full:
                f.write(f"{x:.3f},{y:.3f}\n")
        trajs.append({
            "csv": csv_path,
            "label": f"{name}  ({len(full)} pts, clear≥{wc:.1f}m)",
            "color": col, "x_col": "x", "y_col": "y", "linewidth": 2.0,
        })

    ci = 0
    for name, (a, b) in ROUTES.items():
        print(f"\n=== {name}  {a} -> {b}")
        raw = astar(grid, a, b)
        finalize(name, raw, colors[ci % len(colors)])
        ci += 1

    for name, (spawn, via, end) in ROUTES_VIA.items():
        print(f"\n=== {name}  {spawn} -> {via} -> {end}")
        leg1 = astar(grid, spawn, via)
        leg2 = astar(grid, via, end)
        raw = leg1 + leg2[1:]   # dedup the shared via-point
        finalize(name, raw, colors[ci % len(colors)])
        ci += 1

    with open("/workspace/simulation/isaac/routes/_common/routes.json", "w") as f:
        json.dump(plan, f, indent=2)
    print(f"\nwrote /workspace/simulation/isaac/routes/_common/routes.json")

    # overview plot
    plot_trajectory_map(
        trajectories=trajs,
        output="/workspace/simulation/isaac/routes/_common/routes_plan.png",
        title="Routes 04–09 - planned corner-to-corner roundtrips (≥2m clearance)",
        metrics_lines=[
            f"clearance target: ≥{CLEAR:.1f} m from every object edge  (robot r={ROBOT_R} m)",
            f"04 NW->SE  05 NE->SW  06 N (NW->NE)  07 S (SE->SW)  08 W (NW->SW)  09 E (SE->NE)",
            f"small 2.5m turnaround loop + mirror return along same path",
        ],
        with_obstacles=False,
        with_waypoints=False,
        route="none",
        xlim=(-105, 80),
        ylim=(-45, 42),
    )
    print("saved /workspace/simulation/isaac/routes/_common/routes_plan.png")


if __name__ == "__main__":
    main()
