#!/usr/bin/env python3
"""Drive route point-by-point using web_nav click-to-drive, checking GT after each waypoint"""
import json, time, math
import urllib.request

WEB = "http://localhost:8765"
WPS = json.load(open('/tmp/dense_r1.json'))
TOLERANCE = 3.0  # meters to consider waypoint reached
TIMEOUT = 30     # seconds per waypoint

def get_pos():
    try:
        with open('/tmp/robot_pos.txt') as f:
            parts = f.read().strip().split()
            return float(parts[0]), float(parts[1])
    except:
        return None, None

def send_goal(x, y):
    # world → px (0-1 range) for the web API
    px = (x + 195) / 390
    py = (195 - y) / 390
    urllib.request.urlopen(f'{WEB}/goal?px={px:.4f}&py={py:.4f}', timeout=3)

def stop():
    urllib.request.urlopen(f'{WEB}/stop', timeout=3)

print(f"Driving {len(WPS)} waypoints...")
print(f"Start: ({WPS[0][0]}, {WPS[0][1]})")
print(f"Goal:  ({WPS[-1][0]}, {WPS[-1][1]})")
print()

for i, (wx, wy) in enumerate(WPS):
    rx, ry = get_pos()
    if rx is None:
        print("No position data!")
        break

    dist = math.hypot(wx - rx, wy - ry)
    if dist < TOLERANCE and i < 5:
        # skip nearby starting wps
        continue

    print(f"WP {i}/{len(WPS)-1} → ({wx}, {wy})  [robot at ({rx:.0f}, {ry:.0f}), dist={dist:.0f}m]")
    send_goal(wx, wy)

    t0 = time.time()
    while time.time() - t0 < TIMEOUT:
        time.sleep(0.5)
        rx, ry = get_pos()
        if rx is None:
            continue
        d = math.hypot(wx - rx, wy - ry)
        if d < TOLERANCE:
            print(f"  ✓ reached ({rx:.0f}, {ry:.0f})")
            break
    else:
        rx, ry = get_pos()
        d = math.hypot(wx - rx, wy - ry) if rx else 999
        print(f"  ⚠ timeout at ({rx:.0f}, {ry:.0f}), dist={d:.0f}m — skipping")
        # don't stop, just skip to next wp
        continue

stop()
rx, ry = get_pos()
d_goal = math.hypot(100 - rx, 0 - ry)
print(f"\nFINAL: ({rx:.1f}, {ry:.1f}), {d_goal:.0f}m to goal")
print("SUCCESS" if d_goal < 15 else "PARTIAL")
