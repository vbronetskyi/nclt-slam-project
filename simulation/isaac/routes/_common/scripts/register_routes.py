#!/usr/bin/env python3
"""Merge planned routes (04..09) into /tmp/slam_routes.json (the file that
run_husky_forest.py reads at startup).

Keeps any existing entries untouched; overwrites/adds everything from
_common/routes.json.
"""
import json
from pathlib import Path

PLAN = Path("/workspace/simulation/isaac/routes/_common/routes.json")
SLAM = Path("/tmp/slam_routes.json")

if not PLAN.exists():
    raise SystemExit(f"missing {PLAN} - run generate_routes_04_07.py first")

planned = json.loads(PLAN.read_text())
existing = json.loads(SLAM.read_text()) if SLAM.exists() else {}

for name, pts in planned.items():
    # Store as list of [x, y] pairs (matching the shape run_husky_forest.py expects)
    existing[name] = [[float(x), float(y)] for x, y in pts]

SLAM.write_text(json.dumps(existing, indent=1))
print(f"merged {list(planned.keys())} into {SLAM}")
for k, v in sorted(existing.items()):
    print(f"  {k}: {len(v)} wp")
