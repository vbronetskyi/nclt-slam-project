# _common - shared scripts & artifacts across routes

## scripts/
- `extract_scene_obstacles.py` - reads `/opt/husky_forest_scene.usd`
  directly and dumps every collision prim (ShrubCol, TreeCollision,
  RockCol, RoadsideTrees, Buildings, Props) into `scene_obstacles.json`.
  Authoritative obstacle source - avoids objects missing from
  `/tmp/gazebo_models.json` or `/tmp/usd_obstacles.json`.
- `generate_routes.py` - route planner for corner-to-corner
  roundtrips 04–09. A* on inflated occupancy grid (uses
  `scene_obstacles.json`), Chaikin corner-cutting, smooth hairpin
  turnaround (180° arc + parallel-offset blended return), mirror return.
  Writes `routes.json`, `routes_plan.png`, `drafts/route_*.csv`.
- `register_routes.py` - merges `routes.json` entries into
  `/tmp/slam_routes.json` so `run_husky_forest.py` sees them. Called
  at the top of every per-route `run_teach_single.sh`.

## scene_obstacles.json
Canonical obstacle list (532 objects) extracted from the scene USD.
Used by the route planner.

## routes_04_07.json
Planned routes as `{name: [[x, y], ...]}`. Loaded by
`run_husky_forest.py` via `/tmp/slam_routes.json`-style registration.

## routes_04_07_plan.png
Overview map with all 4 routes + scene objects.

## drafts/
Per-route CSV dumps of planned paths (before they become teach runs).
