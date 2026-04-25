#!/usr/bin/env python3
"""Plan 3 routes through forest, avoiding trees with 1.5m clearance
Uses weighted A* (soft cost near trees) for natural-looking paths
Adds flanking trees so route doesn't look like a cleared corridor
"""
import json, math, heapq, random, re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# config
SDF_PATH = Path("/workspace/simulation/src/ugv_gazebo/worlds/outdoor_terrain.sdf")
ROUTES_JSON = Path("/workspace/simulation/routes/routes.json")
TEXTURE_PATH = Path("/workspace/simulation/src/ugv_gazebo/worlds/terrain_texture.png")
OUTPUT_PNG = Path("/workspace/simulation/routes/routes.png")
OUTPUT_JSON = Path("/workspace/simulation/routes/routes.json")

TREE_CLEARANCE = 1.5   # hard block radius (m)
SOFT_RADIUS = 5.0      # soft cost radius - mild weaving, not too curvy
GRID_RES = 0.5         # m per cell
GRID_XMIN, GRID_XMAX = -130, 130
GRID_YMIN, GRID_YMAX = -80, 80

# r1: forest start -> drop to road -> follow road all the way to village# Road: (-170,-20)->(-150,-25)->(-130,-35)->(-110,-45)->(-90,-50)->(-60,-50)->(-20,-25)->(20,-10)->(60,-3)->(100,0)
ROAD_POINTS = [(-170,20),(-150,25),(-130,35),(-110,45),(-90,50),(-60,50),(-20,25),(20,10),(60,3),(100,0)]

ROUTES = [
    {"key": "route_1", "name": "Forest -> Road -> Village", "color": "yellow",
     "start": (-170, 0), "goal": (100, 0),
     "via_forest": (-20, 0),       # A* thorugh forest staying near y=0
     "via_road_from": (-20, 25)},  # then short connector up to road, follow road east
    {"key": "route_2", "name": "SW -> Road -> Village", "color": "orange",
     "start": (-160, -130), "goal": (100, 0)},
    {"key": "route_3", "name": "NW -> Village", "color": "blue",
     "start": (-100, 150), "goal": (100, 0)},
]

random.seed(42)

# parse trees from SDF
def parse_trees(sdf_path):
    tree = ET.parse(sdf_path)
    trees = []
    for world in tree.getroot().iter("world"):
        for inc in world.findall("include"):
            name_el, pose_el = inc.find("name"), inc.find("pose")
            if name_el is None or pose_el is None:
                continue
            name = name_el.text.strip()
            parts = pose_el.text.strip().split()
            trees.append((name, float(parts[0]), float(parts[1])))
    return trees

# build cost grid
def w2g(wx, wy):
    return int(round((wx - GRID_XMIN) / GRID_RES)), int(round((wy - GRID_YMIN) / GRID_RES))

def g2w(gx, gy):
    return gx * GRID_RES + GRID_XMIN, gy * GRID_RES + GRID_YMIN

def build_cost_grid(trees):
    cols = int((GRID_XMAX - GRID_XMIN) / GRID_RES) + 1
    rows = int((GRID_YMAX - GRID_YMIN) / GRID_RES) + 1
    #base cost 1, blocked=inf, 1-10 near trees (soft gradient)
    cost = np.ones((cols, rows), dtype=np.float32)  # base cost = 1
    blocked = np.zeros((cols, rows), dtype=bool)

    hard_r = int(math.ceil(TREE_CLEARANCE / GRID_RES))
    soft_r = int(math.ceil(SOFT_RADIUS / GRID_RES))

    for _, tx, ty in trees:
        cx, cy = w2g(tx, ty)
        for dx in range(-soft_r, soft_r + 1):
            for dy in range(-soft_r, soft_r + 1):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < cols and 0 <= ny < rows:
                    dist_cells = math.sqrt(dx*dx + dy*dy)
                    dist_m = dist_cells * GRID_RES
                    if dist_m < TREE_CLEARANCE:
                        blocked[nx, ny] = True
                    elif dist_m < SOFT_RADIUS:
                        #quadratic falloff - closer = pricier
                        proximity = 1.0 - (dist_m - TREE_CLEARANCE) / (SOFT_RADIUS - TREE_CLEARANCE)
                        extra = 8.0 * proximity * proximity  # up to 8x cost near trees
                        cost[nx, ny] = max(cost[nx, ny], 1.0 + extra)

    return cost, blocked, cols, rows

#weighted A*
def astar(cost, blocked, start_w, goal_w, cols, rows):
    sx, sy = w2g(*start_w)
    gx, gy = w2g(*goal_w)

    # clamp to grid
    sx, sy = max(0, min(sx, cols-1)), max(0, min(sy, rows-1))
    gx, gy = max(0, min(gx, cols-1)), max(0, min(gy, rows-1))

    # unblock start/goal if they landed on a tree cell
    for px, py in [(sx, sy), (gx, gy)]:
        if blocked[px, py]:
            for r in range(1, 30):
                found = False
                for ddx in range(-r, r+1):
                    for ddy in range(-r, r+1):
                        nnx, nny = px+ddx, py+ddy
                        if 0 <= nnx < cols and 0 <= nny < rows and not blocked[nnx, nny]:
                            if px == sx:
                                sx, sy = nnx, nny
                            else:
                                gx, gy = nnx, nny
                            found = True
                            break
                    if found: break
                if found: break

    open_set = [(0.0, sx, sy)]
    came_from = {}
    g_score = {(sx, sy): 0.0}

    DIAG = math.sqrt(2)
    neighbors = [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,DIAG),(-1,1,DIAG),(1,-1,DIAG),(1,1,DIAG)]

    visited = set()
    while open_set:
        f, cx, cy = heapq.heappop(open_set)
        if (cx, cy) in visited:
            continue
        visited.add((cx, cy))
        if cx == gx and cy == gy:
            path = []
            cur = (gx, gy)
            while cur in came_from:
                path.append(g2w(*cur))
                cur = came_from[cur]
            path.append(g2w(sx, sy))
            path.reverse()
            return path

        for dx, dy, base_cost in neighbors:
            nx, ny = cx+dx, cy+dy
            if 0 <= nx < cols and 0 <= ny < rows and not blocked[nx, ny]:
                step_cost = base_cost * cost[nx, ny]
                ng = g_score[(cx, cy)] + step_cost
                if ng < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = ng
                    h = math.sqrt((nx-gx)**2 + (ny-gy)**2)
                    heapq.heappush(open_set, (ng + h, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)

    print(f"WARNING: A* failed {start_w}->{goal_w}")
    return [start_w, goal_w]

# path simplification
def pt_seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay
    l2 = dx*dx + dy*dy
    if l2 == 0: return math.hypot(px-ax, py-ay)
    t = max(0, min(1, ((px-ax)*dx + (py-ay)*dy) / l2))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))

def seg_clears_trees(p1, p2, trees, clearance):
    for _, tx, ty in trees:
        if pt_seg_dist(tx, ty, p1[0], p1[1], p2[0], p2[1]) < clearance:
            return False
    return True

def simplify_path(raw, trees, clearance, target_n=12):
    """Skip-ahead simplification, keeps tree clearance"""
    if len(raw) <= 2:
        return raw

    result = [raw[0]]
    ci = 0
    while ci < len(raw) - 1:
        # binary search - skip as far ahead as we can
        best = ci + 1
        lo, hi = ci + 1, len(raw) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            if seg_clears_trees(raw[ci], raw[mid], trees, clearance):
                best = mid
                lo = mid + 1
            else:
                hi = mid - 1
        result.append(raw[best])
        ci = best

    # add midpoints if we ended up with too few
    while len(result) < target_n and len(raw) > len(result) * 2:
        expanded = [result[0]]
        for i in range(len(result) - 1):
            p1, p2 = result[i], result[i+1]
            # find closest raw points to p1 and p2
            idx1 = min(range(len(raw)), key=lambda j: math.hypot(raw[j][0]-p1[0], raw[j][1]-p1[1]))
            idx2 = min(range(len(raw)), key=lambda j: math.hypot(raw[j][0]-p2[0], raw[j][1]-p2[1]))
            if idx2 - idx1 > 4:
                mid_idx = (idx1 + idx2) // 2
                mp = raw[mid_idx]
                if (seg_clears_trees(p1, mp, trees, clearance) and
                    seg_clears_trees(mp, p2, trees, clearance)):
                    expanded.append(mp)
            expanded.append(p2)
        if len(expanded) == len(result):
            break  # can't subdivide further
        result = expanded

    return [(round(x, 1), round(y, 1)) for x, y in result]

# flanking trees
def dist_to_route(px, py, pts):
    return min(pt_seg_dist(px, py, pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])
               for i in range(len(pts)-1))

def dist_to_any_tree(px, py, trees):
    return min(math.hypot(px-tx, py-ty) for _, tx, ty in trees)

def generate_flanking_trees(all_routes, existing_trees, n_target=30):
    """Place trees 1.6-5m from route - dense in forest, sparse near village"""
    new = []
    all_t = list(existing_trees)
    attempts = 0
    while len(new) < n_target and attempts < 10000:
        attempts += 1
        route = random.choice(all_routes)
        idx = random.randint(0, len(route) - 2)
        t = random.random()
        bx = route[idx][0] + t * (route[idx+1][0] - route[idx][0])
        by = route[idx][1] + t * (route[idx+1][1] - route[idx][1])

        # density bias: forest=always, open=25%, village=5%   
        if bx > 50 and random.random() > 0.05:
            continue
        elif bx > -20 and random.random() > 0.25:
            continue
        elif bx > -80 and random.random() > 0.70:
            continue

        # perpendicular offset from route
        seg_dx = route[idx+1][0] - route[idx][0]
        seg_dy = route[idx+1][1] - route[idx][1]
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 0.1: continue
        perp_x, perp_y = -seg_dy/seg_len, seg_dx/seg_len
        side = random.choice([-1, 1])
        dist = random.uniform(1.6, 5.0)
        jitter = random.uniform(-3, 3)
        cx = bx + side * dist * perp_x + jitter * seg_dx / seg_len
        cy = by + side * dist * perp_y + jitter * seg_dy / seg_len

        # gotta clear all routes by 1.5m
        too_close = any(dist_to_route(cx, cy, r) < TREE_CLEARANCE for r in all_routes)
        if too_close: continue

        # min 2.5m from existing trees
        if dist_to_any_tree(cx, cy, all_t) < 2.5: continue

        if abs(cx) > 240 or abs(cy) > 240: continue

        tree_type = random.choice(["Pine Tree", "Oak tree"])
        yaw = round(random.uniform(0, 2*math.pi), 3)
        name = f"{tree_type}_flank_{len(new)+1}"
        new.append((name, round(cx, 2), round(cy, 2), yaw))
        all_t.append((name, cx, cy))
    return new

#visualization (same style as web_nav.py)
TEXTURE_PATH = Path("/workspace/simulation/src/ugv_gazebo/worlds/terrain_texture.png")

WEB_MODEL_COLORS = {
    'Oak': (34, 139, 34), 'Pine': (0, 100, 0), 'Rock': (139, 119, 101),
    'Barrel': (255, 165, 0), 'Cone': (255, 69, 0),
    'house': (180, 160, 130), 'collapsed': (140, 90, 80), 'ruin': (140, 90, 80),
    'lamp': (100, 100, 100), 'dumpster': (60, 80, 60),
    'debris': (150, 130, 110), 'jersey': (180, 175, 165),
    'fallen': (70, 45, 25), 'stump': (85, 55, 30), 'Bush': (50, 100, 30),
}

ROUTE_COLORS_NEW = {
    "route_1": (255, 230, 0),
    "route_2": (255, 160, 0),
    "route_3": (0, 180, 255),
}
ROUTE_COLORS_OLD = {
    "route_1": (180, 170, 60),
    "route_2": (200, 120, 40),
    "route_3": (80, 100, 180),
}

def _get_color(name):
    for key, c in WEB_MODEL_COLORS.items():
        if key.lower() in name.lower():
            return c
    return (128, 128, 128)

def visualize(all_models, new_trees_info, old_routes, new_routes, output):
    from PIL import Image as PILImage, ImageDraw

    WORLD_SIZE = 390.0
    MAP_PX = 1000  # same as web
    half = WORLD_SIZE / 2.0

    def w2px(wx, wy):
        return int((wx + half) / WORLD_SIZE * MAP_PX), int((half - wy) / WORLD_SIZE * MAP_PX)

    # background terrain
    try:
        img = PILImage.open(str(TEXTURE_PATH))
        img = img.transpose(PILImage.FLIP_TOP_BOTTOM)
        img = img.resize((MAP_PX, MAP_PX), PILImage.BILINEAR)
    except Exception as e:
        print(f"Texture failed: {e}")
        img = PILImage.new('RGB', (MAP_PX, MAP_PX), (50, 60, 50))
    draw = ImageDraw.Draw(img)

    # model markers
    for name, mx, my, *_ in all_models:
        px, py = w2px(mx, my)
        if not (0 <= px < MAP_PX and 0 <= py < MAP_PX):
            continue
        nl = name.lower()
        color = _get_color(name)
        r = 8
        if 'house' in nl or 'collapsed' in nl or 'ruin' in nl:
            s = 18
            draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(60,40,30), width=2)
        elif 'fallen' in nl:
            draw.line([px-8, py, px+8, py], fill=color, width=3)
        elif 'stump' in nl:
            draw.ellipse([px-4, py-4, px+4, py+4], fill=color, outline=(60,40,20))
        elif 'bush' in nl:
            draw.ellipse([px-5, py-5, px+5, py+5], fill=color, outline=(30,70,20))
        elif ('tree' in nl or 'pine' in nl or 'oak' in nl) and 'fallen' not in nl:
            draw.ellipse([px-r, py-r, px+r, py+r], fill=color, outline=(255,255,255), width=2)
            draw.ellipse([px-2, py-2, px+2, py+2], fill=(255,255,255))
        elif 'rock' in nl:
            draw.polygon([(px,py-r),(px+r,py),(px,py+r),(px-r,py)], fill=color, outline=(255,255,255))
        elif 'jersey' in nl:
            draw.rectangle([px-6, py-2, px+6, py+2], fill=color, outline=(140,140,130))
        elif 'dumpster' in nl:
            draw.rectangle([px-5, py-4, px+5, py+4], fill=color, outline=(40,60,40))
        elif 'debris' in nl:
            draw.polygon([(px,py-4),(px+5,py+2),(px-5,py+2)], fill=color, outline=(120,110,90))
        else:
            s = r // 2 + 1
            draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(255,255,255), width=2)

    # flanking trees (red outline)
    for name, nx, ny, *_ in new_trees_info:
        px, py = w2px(nx, ny)
        r = 8
        color = _get_color(name)
        draw.ellipse([px-r, py-r, px+r, py+r], fill=color, outline=(255, 60, 60), width=2)
        draw.ellipse([px-2, py-2, px+2, py+2], fill=(255, 200, 200))

    # old routes (dashed, faded)
    for key in ["route_1", "route_2", "route_3"]:
        rd = old_routes[key]
        pts = [rd["start"]] + rd["waypoints"] + [rd["goal"]]
        px_pts = [w2px(p[0], p[1]) for p in pts]
        c = ROUTE_COLORS_OLD[key]
        for i in range(len(px_pts)-1):
            x1, y1 = px_pts[i]
            x2, y2 = px_pts[i+1]
            length = math.hypot(x2-x1, y2-y1)
            if length < 1: continue
            dash = 8
            n = max(1, int(length / (dash * 2)))
            for d in range(n):
                t1 = d * 2 * dash / length
                t2 = min(1.0, (d * 2 + 1) * dash / length)
                draw.line([(int(x1+(x2-x1)*t1), int(y1+(y2-y1)*t1)),
                           (int(x1+(x2-x1)*t2), int(y1+(y2-y1)*t2))], fill=c, width=2)

    # new routes (solid, bright)
    for key in ["route_1", "route_2", "route_3"]:
        rd = new_routes[key]
        pts = [rd["start"]] + [tuple(w) for w in rd["waypoints"]] + [rd["goal"]]
        px_pts = [w2px(p[0], p[1]) for p in pts]
        c = ROUTE_COLORS_NEW[key]
        for i in range(len(px_pts)-1):
            draw.line([px_pts[i], px_pts[i+1]], fill=c, width=3)
        for pp in px_pts:
            draw.ellipse([pp[0]-4, pp[1]-4, pp[0]+4, pp[1]+4], fill=c, outline=(0,0,0), width=1)

    # start markers (green)
    for sx, sy in [(-170, 0), (-160, -130), (-100, 150)]:
        px, py = w2px(sx, sy)
        draw.ellipse([px-8, py-8, px+8, py+8], fill=(0, 200, 0), outline=(255,255,255), width=2)

    # goal (red X)
    gx, gy = w2px(100, 0)
    r = 12
    draw.line([gx-r, gy-r, gx+r, gy+r], fill=(255, 0, 0), width=3)
    draw.line([gx-r, gy+r, gx+r, gy-r], fill=(255, 0, 0), width=3)
    draw.ellipse([gx-r, gy-r, gx+r, gy+r], outline=(255, 0, 0), width=2)

    #legend
    y0 = 10
    legend = [
        ("R1 new (forest)", ROUTE_COLORS_NEW["route_1"]),
        ("R2 new (south)", ROUTE_COLORS_NEW["route_2"]),
        ("R3 new (north)", ROUTE_COLORS_NEW["route_3"]),
        ("Old routes", (150, 150, 100)),
        ("Tree", (34, 139, 34)),
        ("New tree (flank)", (34, 139, 34)),
        ("Building", (180, 160, 130)),
        ("Ruin", (140, 90, 80)),
        ("Goal", (255, 0, 0)),
    ]
    for label, c in legend:
        draw.rectangle([10, y0, 22, y0+12], fill=c)
        if 'flank' in label:
            draw.rectangle([10, y0, 22, y0+12], outline=(255, 60, 60), width=2)
        draw.text((26, y0), label, fill=(255, 255, 255))
        y0 += 16

    #scale + compass   
    bar_m = 10
    bar_px = int(bar_m / WORLD_SIZE * MAP_PX)
    draw.rectangle([10, MAP_PX-25, 10+bar_px, MAP_PX-20], fill=(255,255,255))
    draw.text((10, MAP_PX-18), f"{bar_m}m", fill=(255,255,255))
    draw.text((MAP_PX//2-10, 2), "N", fill=(200,200,200))
    draw.text((MAP_PX//2-10, MAP_PX-16), "S", fill=(200,200,200))
    draw.text((2, MAP_PX//2-6), "W", fill=(200,200,200))
    draw.text((MAP_PX-12, MAP_PX//2-6), "E", fill=(200,200,200))

    img.save(str(output), optimize=True)
    print(f"Saved: {output} ({img.size[0]}x{img.size[1]})")

def main():
    print("=== UGV Route Planner v2 (weighted A*) ===")
    trees = parse_trees(SDF_PATH)
    print(f"Parsed {len(trees)} trees/bushes")

    print("Building weighted cost grid...")
    cost, blocked, cols, rows = build_cost_grid(trees)
    print(f"Grid: {cols}x{rows}, blocked: {np.sum(blocked)}, soft-cost cells: {np.sum(cost > 1.5)}")

    new_routes_data = {}
    new_routes_pts = []

    for rdef in ROUTES:
        print(f"\n--- {rdef['key']}: {rdef['start']} -> {rdef['goal']} ---")

        if 'via_road_from' in rdef:
            junction = rdef['via_road_from']

            if 'via_forest' in rdef:
                # three-phase: A* forest -> connector -> road
                forest_end = rdef['via_forest']
                print(f"  Phase 1: A* {rdef['start']} -> forest end {forest_end}")
                raw1 = astar(cost, blocked, rdef['start'], forest_end, cols, rows)
                print(f"  Phase 1 raw: {len(raw1)} pts")
                simplified1 = simplify_path(raw1, trees, TREE_CLEARANCE, target_n=8)
                simplified1[0] = rdef['start']
                simplified1[-1] = forest_end

                print(f"  Phase 2: connector {forest_end} -> road {junction}")
                raw2 = astar(cost, blocked, forest_end, junction, cols, rows)
                simplified2 = simplify_path(raw2, trees, TREE_CLEARANCE, target_n=4)
                simplified2[0] = forest_end
                simplified2[-1] = junction
            else:
                print(f"  Phase 1: A* {rdef['start']} -> road junction {junction}")
                raw1 = astar(cost, blocked, rdef['start'], junction, cols, rows)
                print(f"  Phase 1 raw: {len(raw1)} pts")
                simplified1 = simplify_path(raw1, trees, TREE_CLEARANCE, target_n=8)
                simplified1[0] = rdef['start']
                simplified1[-1] = junction
                simplified2 = [junction]

            # follow road from junction to goal
            road_segment = []
            found_start = False
            for rp in ROAD_POINTS:
                if rp == tuple(junction) or (abs(rp[0]-junction[0]) < 5 and abs(rp[1]-junction[1]) < 5):
                    found_start = True
                if found_start:
                    road_segment.append(rp)
            if not road_segment:
                road_segment = [junction, rdef['goal']]
            print(f"  Phase 3: road segment {len(road_segment)} pts")

            simplified = simplified1[:-1] + simplified2[:-1] + road_segment
            simplified[-1] = rdef['goal']
        else:
            raw = astar(cost, blocked, rdef['start'], rdef['goal'], cols, rows)
            print(f"  A* raw: {len(raw)} pts")
            simplified = simplify_path(raw, trees, TREE_CLEARANCE, target_n=12)
            simplified[0] = rdef['start']
            simplified[-1] = rdef['goal']
        print(f"  Simplified: {len(simplified)} waypoints")
        for wp in simplified:
            print(f"    {wp}")

        # verify clearance
        min_c = float('inf')
        for _, tx_, ty_ in trees:
            pts_list = simplified
            for i in range(len(pts_list)-1):
                d = pt_seg_dist(tx_, ty_, pts_list[i][0], pts_list[i][1],
                               pts_list[i+1][0], pts_list[i+1][1])
                min_c = min(min_c, d)
        print(f"  Min clearance: {min_c:.2f}m {'OK' if min_c >= 1.5 else 'VIOLATION!'}")

        new_routes_pts.append(simplified)
        new_routes_data[rdef['key']] = {
            "name": rdef['name'],
            "color": rdef['color'],
            "start": list(rdef['start']),
            "waypoints": [list(p) for p in simplified[1:-1]],
            "goal": list(rdef['goal']),
        }

    # no flanking trees needed - routes weave between existing ones
    new_trees = []

    # old routes for comparison overlay
    with open(ROUTES_JSON) as f:
        old_routes = json.load(f)
    if "road" in old_routes:
        new_routes_data["road"] = old_routes["road"]

    # save
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(new_routes_data, f, indent=2)
    print(f"\nSaved routes: {OUTPUT_JSON}")

    # visualize (all SDF models for buildings etc.)
    all_models = parse_trees(SDF_PATH)  # includes buildings, dumpsters, etc
    visualize(all_models, new_trees, old_routes, new_routes_data, OUTPUT_PNG)

    # print SDF snippets for new trees
    print("\n=== SDF snippets for new trees ===")
    for name, x, y, yaw in new_trees:
        model_type = "Oak tree" if "Oak" in name else "Pine Tree"
        uri = f"https://fuel.gazebosim.org/1.0/OpenRobotics/models/{model_type}"
        print(f"""    <include>
      <name>{name}</name>
      <uri>{uri}</uri>
      <static>true</static>
      <pose>{x} {y} 0.0 0 0 {yaw}</pose>
    </include>""")

    print("\nDone!")

if __name__ == "__main__":
    main()
