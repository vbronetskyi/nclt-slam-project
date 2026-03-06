#!/usr/bin/env python3
"""Render 2D maps in web_nav style (terrain_texture + model markers)
Generates: world_routes.png, world_scenario.png, map_routes.png
"""
import math, json
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
from pathlib import Path

SDF = Path("/workspace/simulation/src/ugv_gazebo/worlds/outdoor_terrain.sdf")
TEX = Path("/workspace/simulation/src/ugv_gazebo/worlds/terrain_texture.png")
ROUTES = Path("/workspace/simulation/routes/routes.json")
OUT = Path("/workspace/simulation")

MODEL_COLORS = {
    'Oak': (34, 139, 34), 'Pine': (0, 100, 0), 'Rock': (139, 119, 101),
    'Barrel': (255, 165, 0), 'Cone': (255, 69, 0),
    'house': (180, 160, 130), 'collapsed': (140, 90, 80), 'ruin': (140, 90, 80),
    'lamp': (100, 100, 100), 'dumpster': (60, 80, 60),
    'debris': (150, 130, 110), 'jersey': (180, 175, 165),
    'fallen': (70, 45, 25), 'stump': (85, 55, 30), 'Bush': (50, 100, 30),
}

ROUTE_COLORS = {
    "route_1": (255, 230, 0),
    "route_2": (255, 160, 0),
    "route_3": (0, 180, 255),
}

def _color(name):
    for k, c in MODEL_COLORS.items():
        if k.lower() in name.lower():
            return c
    return (128, 128, 128)

def load_models():
    models = []
    tree = ET.parse(SDF)
    for inc in tree.iter('include'):
        n, p = inc.find('name'), inc.find('pose')
        if n is not None and p is not None:
            parts = [float(x) for x in p.text.split()]
            yaw = parts[5] if len(parts) > 5 else 0.0
            models.append((n.text, parts[0], parts[1], yaw))
    return models

def load_terrain(sz):
    img = PILImage.open(str(TEX))
    # no flip needed - texture matches Gazebo world coords
    return img.resize((sz, sz), PILImage.BILINEAR)

def draw_models(draw, models, w2px, sz):
    """Draw all models (same style as web_nav)"""
    for item in models:
        name, mx, my = item[0], item[1], item[2]
        yaw = item[3] if len(item) > 3 else 0.0
        px, py = w2px(mx, my)
        if not (0 <= px < sz and 0 <= py < sz):
            continue
        nl = name.lower()
        color = _color(name)
        r = 8
        if 'house' in nl or 'collapsed' in nl or 'ruin' in nl:
            s = 18
            draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(60,40,30), width=2)
        elif 'fallen' in nl:
            import math as _m
            dx = int(10 * _m.cos(yaw)); dy = int(-10 * _m.sin(yaw))
            draw.line([px-dx, py-dy, px+dx, py+dy], fill=(90, 60, 30), width=3)
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

def draw_routes(draw, routes_data, w2px):
    for key in ["route_1", "route_2", "route_3"]:
        if key not in routes_data:
            continue
        rd = routes_data[key]
        pts = [rd["start"]] + [tuple(w) if isinstance(w, list) else w for w in rd["waypoints"]] + [rd["goal"]]
        px_pts = [w2px(p[0], p[1]) for p in pts]
        c = ROUTE_COLORS[key]
        for i in range(len(px_pts)-1):
            draw.line([px_pts[i], px_pts[i+1]], fill=c, width=3)
        for pp in px_pts:
            draw.ellipse([pp[0]-4, pp[1]-4, pp[0]+4, pp[1]+4], fill=c, outline=(0,0,0), width=1)

    # route start markers
    for key in ["route_1", "route_2", "route_3"]:
        if key not in routes_data:
            continue
        sx, sy = routes_data[key]["start"]
        px, py = w2px(sx, sy)
        draw.ellipse([px-8, py-8, px+8, py+8], fill=(0,200,0), outline=(255,255,255), width=2)

    # goal
    gx, gy = w2px(100, 0)
    r = 12
    draw.line([gx-r, gy-r, gx+r, gy+r], fill=(255,0,0), width=3)
    draw.line([gx-r, gy+r, gx+r, gy-r], fill=(255,0,0), width=3)
    draw.ellipse([gx-r, gy-r, gx+r, gy+r], outline=(255,0,0), width=2)

def add_compass_scale(draw, sz, world_size, bar_m=10):
    bar_px = int(bar_m / world_size * sz)
    draw.rectangle([10, sz-25, 10+bar_px, sz-20], fill=(255,255,255))
    draw.text((10, sz-18), f"{bar_m}m", fill=(255,255,255))
    draw.text((sz//2-10, 2), "N", fill=(200,200,200))
    draw.text((sz//2-10, sz-16), "S", fill=(200,200,200))
    draw.text((2, sz//2-6), "W", fill=(200,200,200))
    draw.text((sz-12, sz//2-6), "E", fill=(200,200,200))

# world_routes.png -- full map with routes
def render_world_routes(models, routes_data):
    WS, SZ = 390.0, 1000
    half = WS / 2.0
    def w2px(wx, wy):
        return int((wx+half)/WS*SZ), int((half-wy)/WS*SZ)

    img = load_terrain(SZ)
    draw = ImageDraw.Draw(img)
    draw_models(draw, models, w2px, SZ)
    draw_routes(draw, routes_data, w2px)

    # legend
    y0 = 10
    for label, c in [("R1 Forest->Village", (255,230,0)), ("R2 SW->Village", (255,160,0)),
                      ("R3 NW->Village", (0,180,255)), ("Tree", (34,139,34)),
                      ("Building", (180,160,130)), ("Ruin", (140,90,80)), ("Goal", (255,0,0))]:
        draw.rectangle([10, y0, 22, y0+12], fill=c)
        draw.text((26, y0), label, fill=(255,255,255))
        y0 += 16

    add_compass_scale(draw, SZ, WS)
    out = OUT / "world_routes.png"
    img.save(str(out), optimize=True)
    print(f"Saved: {out}")

# world_scenario.png -- zones + labels + routes
def render_world_scenario(models, routes_data):
    WS, SZ = 390.0, 1000
    half = WS / 2.0
    def w2px(wx, wy):
        return int((wx+half)/WS*SZ), int((half-wy)/WS*SZ)

    img = load_terrain(SZ)
    draw = ImageDraw.Draw(img, 'RGBA')

    # zone overlays (semi-transparent rects)
    # forest: x < -30
    fx1, fy1 = w2px(-195, 195)
    fx2, fy2 = w2px(-30, -195)
    draw.rectangle([fx1, fy1, fx2, fy2], fill=(0, 80, 0, 40), outline=(0, 120, 0, 100), width=2)

    # open field: -30 < x < 60
    ox1, oy1 = w2px(-30, 195)
    ox2, oy2 = w2px(60, -195)
    draw.rectangle([ox1, oy1, ox2, oy2], fill=(180, 160, 80, 30), outline=(180, 160, 80, 80), width=2)

    # village: x > 60
    vx1, vy1 = w2px(60, 195)
    vx2, vy2 = w2px(195, -195)
    draw.rectangle([vx1, vy1, vx2, vy2], fill=(160, 120, 80, 40), outline=(160, 120, 80, 100), width=2)

    # switch back to RGB for solid stuff
    draw = ImageDraw.Draw(img)
    draw_models(draw, models, w2px, SZ)
    draw_routes(draw, routes_data, w2px)

    # zone labels
    zones = [("DENSE FOREST", -120, 160), ("OPEN FIELD", 10, 160), ("VILLAGE", 100, 160),
             ("Spawn R1", -170, 8), ("Spawn R2", -155, -122), ("Spawn R3", -95, 155)]
    for label, zx, zy in zones:
        px, py = w2px(zx, zy)
        draw.text((px, py), label, fill=(255, 255, 255))

    # legend
    y0 = 10
    for label, c in [("Forest zone", (0,120,0)), ("Open field", (180,160,80)),
                      ("Village zone", (160,120,80)), ("R1", (255,230,0)),
                      ("R2", (255,160,0)), ("R3", (0,180,255)),
                      ("Tree", (34,139,34)), ("Building", (180,160,130)), ("Goal", (255,0,0))]:
        draw.rectangle([10, y0, 22, y0+12], fill=c)
        draw.text((26, y0), label, fill=(255,255,255))
        y0 += 16

    add_compass_scale(draw, SZ, WS)
    out = OUT / "world_scenario.png"
    img.save(str(out), optimize=True)
    print(f"Saved: {out}")

# map_routes.png -- zoomed center area
def render_map_routes(models, routes_data):
    # crop to the interesting area
    XMIN, XMAX = -180, 120
    YMIN, YMAX = -150, 170
    WS = 220.0
    SZ = 1200  # higher res for zoom
    half = WS / 2.0

    # load full terrain then crop out
    full_sz = int(SZ * WS / (XMAX - XMIN))
    full_img = load_terrain(full_sz)

    # crop region in px
    def w2full(wx, wy):
        return int((wx+half)/WS*full_sz), int((half-wy)/WS*full_sz)

    left, top = w2full(XMIN, YMAX)
    right, bot = w2full(XMAX, YMIN)
    img = full_img.crop((left, top, right, bot)).resize((SZ, SZ), PILImage.BILINEAR)
    draw = ImageDraw.Draw(img)

    # coord transform for cropped region
    def w2px(wx, wy):
        px = int((wx - XMIN) / (XMAX - XMIN) * SZ)
        py = int((YMAX - wy) / (YMAX - YMIN) * SZ)
        return px, py

    draw_models(draw, models, w2px, SZ)
    draw_routes(draw, routes_data, w2px)

    # legend
    y0 = 10
    for label, c in [("R1 Forest->Village", (255,230,0)), ("R2 SW->Village", (255,160,0)),
                      ("R3 NW->Village", (0,180,255)), ("Tree", (34,139,34)),
                      ("Building", (180,160,130)), ("Goal", (255,0,0))]:
        draw.rectangle([10, y0, 22, y0+12], fill=c)
        draw.text((26, y0), label, fill=(255,255,255))
        y0 += 16

    # scale bar for zoomed map
    bar_m = 20
    bar_px = int(bar_m / (XMAX - XMIN) * SZ)
    draw.rectangle([10, SZ-25, 10+bar_px, SZ-20], fill=(255,255,255))
    draw.text((10, SZ-18), f"{bar_m}m", fill=(255,255,255))
    draw.text((SZ//2-10, 2), "N", fill=(200,200,200))
    draw.text((SZ//2-10, SZ-16), "S", fill=(200,200,200))

    out = OUT / "map_routes.png"
    img.save(str(out), optimize=True)
    print(f"Saved: {out}")

# world_realistic.png -- clean world overview, no routes
def render_world_realistic(models):
    """Full world map with all objects and terrain, no routes"""
    WS, SZ = 390.0, 1200  # higher res
    half = WS / 2.0
    def w2px(wx, wy):
        return int((wx+half)/WS*SZ), int((half-wy)/WS*SZ)

    img = load_terrain(SZ)
    draw = ImageDraw.Draw(img)

    # scaled up markers for higher res
    for name, mx, my in models:
        px, py = w2px(mx, my)
        if not (0 <= px < SZ and 0 <= py < SZ):
            continue
        nl = name.lower()
        color = _color(name)
        r = 10
        if 'house' in nl or 'collapsed' in nl or 'ruin' in nl:
            s = 22
            draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(60,40,30), width=2)
        elif 'fallen' in nl:
            draw.line([px-10, py, px+10, py], fill=color, width=4)
        elif 'stump' in nl:
            draw.ellipse([px-5, py-5, px+5, py+5], fill=color, outline=(60,40,20))
        elif 'bush' in nl:
            draw.ellipse([px-6, py-6, px+6, py+6], fill=color, outline=(30,70,20))
        elif ('tree' in nl or 'pine' in nl or 'oak' in nl) and 'fallen' not in nl:
            draw.ellipse([px-r, py-r, px+r, py+r], fill=color, outline=(255,255,255), width=2)
            draw.ellipse([px-2, py-2, px+2, py+2], fill=(255,255,255))
        elif 'rock' in nl:
            draw.polygon([(px,py-r),(px+r,py),(px,py+r),(px-r,py)], fill=color, outline=(255,255,255))
        elif 'jersey' in nl:
            draw.rectangle([px-7, py-3, px+7, py+3], fill=color, outline=(140,140,130))
        elif 'dumpster' in nl:
            draw.rectangle([px-6, py-5, px+6, py+5], fill=color, outline=(40,60,40))
        elif 'debris' in nl:
            draw.polygon([(px,py-5),(px+6,py+3),(px-6,py+3)], fill=color, outline=(120,110,90))
        else:
            s = 5
            draw.rectangle([px-s, py-s, px+s, py+s], fill=color, outline=(255,255,255), width=2)

    # zone labels
    for label, zx, zy in [("DENSE FOREST", -130, 170), ("OPEN FIELD", 5, 170), ("VILLAGE", 95, 170)]:
        px, py = w2px(zx, zy)
        draw.text((px, py), label, fill=(255, 255, 255))

    # legend
    y0 = 10
    for label, c in [("Oak tree", (34,139,34)), ("Pine tree", (0,100,0)),
                      ("Bush", (50,100,30)), ("Rock", (139,119,101)),
                      ("House", (180,160,130)), ("Ruin", (140,90,80)),
                      ("Dumpster", (60,80,60)), ("Debris", (150,130,110)),
                      ("Barrel", (255,165,0)), ("Cone", (255,69,0))]:
        draw.rectangle([10, y0, 22, y0+12], fill=c)
        draw.text((26, y0), label, fill=(255,255,255))
        y0 += 16

    add_compass_scale(draw, SZ, WS, bar_m=20)
    out = OUT / "world_realistic.png"
    img.save(str(out), optimize=True)
    print(f"Saved: {out}")

def main():
    models = load_models()
    print(f"Loaded {len(models)} models from SDF")

    with open(ROUTES) as f:
        routes_data = json.load(f)

    render_world_routes(models, routes_data)
    render_world_scenario(models, routes_data)
    render_map_routes(models, routes_data)
    render_world_realistic(models)
    print("All maps regenerated!")

if __name__ == "__main__":
    main()
