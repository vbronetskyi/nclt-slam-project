#!/usr/bin/env python3
"""
Render 2D world map with routes, objects, and terrain

Generates map images for docs and route planning
Uses heightmap + SDF file

Usage:
    python3 render_map.py [--output map.png] [--size 1600] [--routes]
"""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

# paths
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
WORLDS_DIR = PROJECT_DIR / "src" / "ugv_gazebo" / "worlds"
ROUTES_PATH = PROJECT_DIR / "routes" / "routes.json"
HEIGHTMAP_PATH = WORLDS_DIR / "heightmap.png"
SDF_PATH = WORLDS_DIR / "outdoor_terrain.sdf"

# object colors
MODEL_COLORS = {
    'oak': (34, 139, 34),
    'pine': (0, 100, 0),
    'rock': (139, 119, 101),
    'barrel': (255, 165, 0),
    'cone': (255, 69, 0),
    'house': (180, 160, 130),
    'collapsed': (140, 90, 80),
    'lamp': (100, 100, 100),
    'dumpster': (80, 120, 70),
    'hydrant': (200, 40, 30),
    'debris': (150, 130, 110),
    'depot': (160, 140, 100),
    'bush': (50, 120, 40),
    'stump': (100, 80, 50),
    'fallen': (80, 60, 30),
}

# route colors
ROUTE_COLORS = {
    'R1': (255, 215, 0),       # gold/yellow
    'R2': (255, 140, 0),       # dark orange
    'R3': (65, 105, 225),      # royal blue
}


def load_heightmap_terrain(terrain_size, map_px):
    """Load heightmap and render as map background"""
    try:
        raw = Image.open(HEIGHTMAP_PATH)
        hm = np.array(raw, dtype=np.float64) / 65535.0
        rgb = np.zeros((hm.shape[0], hm.shape[1], 3), dtype=np.uint8)

        low_mask = hm < 0.5
        high_mask = ~low_mask
        t_low = hm[low_mask] * 2.0
        t_high = (hm[high_mask] - 0.5) * 2.0

        rgb[low_mask, 0] = (60 + 80 * t_low).astype(np.uint8)
        rgb[low_mask, 1] = (120 + 40 * t_low).astype(np.uint8)
        rgb[low_mask, 2] = (40 + 10 * t_low).astype(np.uint8)

        rgb[high_mask, 0] = (140 - 10 * t_high).astype(np.uint8)
        rgb[high_mask, 1] = (160 - 70 * t_high).astype(np.uint8)
        rgb[high_mask, 2] = (50 - 10 * t_high).astype(np.uint8)

        # no flip needed - Y matches via world_to_px
        return Image.fromarray(rgb).resize((map_px, map_px), Image.BILINEAR)
    except Exception as e:
        print(f"[warn] Heightmap not found: {e}")
        return Image.new('RGB', (map_px, map_px), (50, 60, 50))


def load_sdf_models(sdf_path):
    """Parse SDF for object positions"""
    models = []
    try:
        tree = ET.parse(sdf_path)
        for inc in tree.iter('include'):
            name_el = inc.find('name')
            pose_el = inc.find('pose')
            if name_el is not None and pose_el is not None:
                p = [float(x) for x in pose_el.text.split()]
                n = name_el.text
                color = (128, 128, 128)
                for key, c in MODEL_COLORS.items():
                    if key in n.lower():
                        color = c
                        break
                models.append((n, p[0], p[1], color))
    except Exception as e:
        print(f"[warn] SDF parse error: {e}")
    return models


def load_routes(routes_path):
    """Load routes from JSON"""
    try:
        with open(routes_path) as f:
            return json.load(f)
    except Exception as e:
        print(f"[warn] Routes not found: {e}")
        return None


def render_map(terrain_size=250.0, map_px=1600, show_routes=True, output_path=None):
    """Render full map with objects and routes"""
    half = terrain_size / 2.0

    def world_to_px(wx, wy):
        px = int((wx + half) / terrain_size * map_px)
        py = int((half - wy) / terrain_size * map_px)
        return px, py

    # 1. terrain background
    img = load_heightmap_terrain(terrain_size, map_px)
    draw = ImageDraw.Draw(img)

    # 2. road
    road_points = [(-170, -20), (-150, -25), (-130, -35), (-110, -45),
                   (-90, -50), (-60, -50), (-20, -25), (20, -10), (60, -3), (100, 0)]
    road_px = [world_to_px(x, y) for x, y in road_points]
    if len(road_px) > 1:
        for i in range(len(road_px) - 1):
            draw.line([road_px[i], road_px[i+1]], fill=(140, 120, 80), width=8)

    # 3. objects from SDF
    models = load_sdf_models(SDF_PATH)
    for name, mx, my, color in models:
        px, py = world_to_px(mx, my)
        if 0 <= px < map_px and 0 <= py < map_px:
            r = 6
            nl = name.lower()
            if 'house' in nl or 'collapsed' in nl or 'depot' in nl:
                s = 14
                outline = (60, 40, 30)
                if 'collapsed' in nl:
                    outline = (100, 50, 40)
                draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=outline, width=2)
            elif 'tree' in nl or 'pine' in nl or 'oak' in nl:
                draw.ellipse([px-r, py-r, px+r, py+r], fill=color, outline=(200, 200, 200))
            elif 'rock' in nl:
                draw.polygon([(px, py-r), (px+r, py), (px, py+r), (px-r, py)],
                             fill=color, outline=(200, 200, 200))
            elif 'bush' in nl:
                draw.ellipse([px-4, py-4, px+4, py+4], fill=color)
            elif 'stump' in nl:
                draw.ellipse([px-3, py-3, px+3, py+3], fill=color)
            else:
                s = 4
                draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(200, 200, 200))

    # 4. routes
    if show_routes:
        routes_data = load_routes(ROUTES_PATH)
        if routes_data and 'routes' in routes_data:
            for route_id, route in routes_data['routes'].items():
                color = ROUTE_COLORS.get(route_id, (200, 200, 200))
                start = route['start']
                goal = route['goal']
                waypoints = route.get('waypoints', [])

                points = [world_to_px(start[0], start[1])]
                for wp in waypoints:
                    if isinstance(wp, dict):
                        points.append(world_to_px(wp['x'], wp.get('y', 0)))
                    else:
                        points.append(world_to_px(wp[0], wp[1]))
                points.append(world_to_px(goal[0], goal[1]))

                for i in range(len(points) - 1):
                    draw.line([points[i], points[i+1]], fill=color, width=3)

                sx, sy = points[0]
                draw.ellipse([sx-8, sy-8, sx+8, sy+8], fill=color, outline=(255, 255, 255), width=2)
                draw.text((sx+10, sy-6), route_id, fill=color)

            goal = routes_data.get('goal', [100, 0])
            gx, gy = world_to_px(goal[0], goal[1])
            r = 12
            draw.line([gx-r, gy-r, gx+r, gy+r], fill=(255, 0, 0), width=3)
            draw.line([gx-r, gy+r, gx+r, gy-r], fill=(255, 0, 0), width=3)
            draw.ellipse([gx-r, gy-r, gx+r, gy+r], outline=(255, 0, 0), width=2)
            draw.text((gx+14, gy-6), "GOAL", fill=(255, 0, 0))

    # 5. zone labels
    zones = [
        ((-140, 80), "Dense Forest"),
        ((-10, 80), "Open Field"),
        ((90, 80), "Village"),
    ]
    for (zx, zy), label in zones:
        px, py = world_to_px(zx, zy)
        draw.text((px-30, py), label, fill=(255, 255, 255))

    # 6. legend
    y0 = 10
    items = [
        ("Road", (140, 120, 80)),
        ("R1 (forest)", ROUTE_COLORS['R1']),
        ("R2 (south)", ROUTE_COLORS['R2']),
        ("R3 (north)", ROUTE_COLORS['R3']),
        ("Tree", (34, 139, 34)),
        ("Rock", (139, 119, 101)),
        ("Building", (180, 160, 130)),
        ("Ruin", (140, 90, 80)),
        ("Barrel/Cone", (255, 165, 0)),
        ("Goal", (255, 0, 0)),
    ]
    for label, c in items:
        draw.rectangle([10, y0, 22, y0 + 12], fill=c)
        draw.text((26, y0), label, fill=(255, 255, 255))
        y0 += 16

    # 7. scale bar
    bar_m = 50
    bar_px = int(bar_m / terrain_size * map_px)
    draw.rectangle([10, map_px - 30, 10 + bar_px, map_px - 25], fill=(255, 255, 255))
    draw.text((10, map_px - 22), f"{bar_m}m", fill=(255, 255, 255))

    # 8. compass
    draw.text((map_px // 2 - 5, 5), "N", fill=(220, 220, 220))
    draw.text((map_px // 2 - 5, map_px - 18), "S", fill=(220, 220, 220))
    draw.text((5, map_px // 2 - 6), "W", fill=(220, 220, 220))
    draw.text((map_px - 14, map_px // 2 - 6), "E", fill=(220, 220, 220))

    if output_path is None:
        output_path = PROJECT_DIR / "map_routes.png"
    img.save(str(output_path), optimize=True)
    print(f"Map saved: {output_path} ({img.size[0]}x{img.size[1]})")
    return img


def main():
    parser = argparse.ArgumentParser(description="Render world map with routes")
    parser.add_argument("--output", type=str, default=None, help="Output PNG path")
    parser.add_argument("--size", type=int, default=1600, help="Map size in pixels")
    parser.add_argument("--terrain", type=float, default=390.0, help="Terrain size in meters")
    parser.add_argument("--no-routes", action="store_true", help="Hide routes")
    args = parser.parse_args()

    output = Path(args.output) if args.output else None
    render_map(
        terrain_size=args.terrain,
        map_px=args.size,
        show_routes=not args.no_routes,
        output_path=output,
    )


if __name__ == "__main__":
    main()
