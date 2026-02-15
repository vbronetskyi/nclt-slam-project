# Path Planning Based on ORB-SLAM3 RGB-D: Occupancy Grid and A*

## Overview

This module demonstrates a complete **autonomous navigation pipeline**: from raw RGB-D depth data to obstacle-avoiding path planning. It uses the results of ORB-SLAM3 RGB-D on the `garden_large_day` recording from the ROVER dataset.

**Input data:**
- Estimated ORB-SLAM3 poses (11,789 frames, TUM format)
- Depth images from Intel RealSense D435i (640x480, uint16, mm)
- GT trajectory from Leica Total Station (1,099 points)

**Output:** a 3-panel visualization with the occupancy grid, A* routes, and the point cloud.

---

## Pipeline

### Step 1: Data Loading

ORB-SLAM3 poses in TUM format (`timestamp tx ty tz qx qy qz qw`), D435i depth images, and the GT trajectory are loaded. Each pose is a 4x4 transformation matrix (R|t) describing the camera position in the world coordinate system.

### Step 2: Back-projection of Depth to 3D Points

For every **10th frame** (1,179 out of 11,789), every **4th pixel** of the depth image is back-projected into 3D:

```
X_cam = (u - cx) * d / fx
Y_cam = (v - cy) * d / fy
Z_cam = d

P_world = R * P_cam + t
```

**D435i camera parameters:**
| Parameter | Value |
|-----------|-------|
| fx, fy | 596.20, 593.14 |
| cx, cy | 327.05, 245.16 |
| Resolution | 640 x 480 |
| Depth filter | 0.1 -- 6.0 m |

**Result:** 15.6 million 3D points in the ORB-SLAM3 world coordinate system.

### Step 3: Height Classification (Floor vs. Obstacle)

**Key consideration:** ORB-SLAM3 uses a camera coordinate system where **Y points downward**. The X-Z plane is horizontal and Y is the vertical axis.

Because the ORB-SLAM3 world frame is **not gravity-aligned** (the origin is the first camera frame), absolute height thresholds do not work. Instead, classification is performed **relative to the camera position for each frame**:

```
y_rel = y_point - y_camera
```

| Category | Condition | Explanation |
|----------|-----------|-------------|
| **Floor** | y_rel >= +0.10 m | Points below the camera - ground |
| **Obstacle** | -1.0 < y_rel < +0.10 m | Points at camera level - walls, bushes |
| **Ignore** | Everything else | Too high (trees, sky) |

**Empirical justification:** the median y_rel for ground points is approximately +0.20 m (the D435i camera sits ~20-30 cm above the ground on the ROVER robot).

**Classification result:**
- Floor: 7.84 million points (50.3%)
- Obstacle: 6.63 million points (42.6%)
- Ignore: 1.10 million points (7.1%)

### Step 4: Building the 2D Occupancy Grid

The classified 3D points are projected onto the X-Z plane at a resolution of **5 cm/cell**:

| Parameter | Value |
|-----------|-------|
| Resolution | 0.05 m (5 cm) |
| Grid size | 453 x 553 cells |
| Coverage | X: [-6.7, 15.9] m, Z: [-16.0, 11.6] m |

**Cell classification rules:**
- A minimum of 3 total hits is required to classify a cell
- **Free (0):** < 5 obstacle hits - traversable
- **Occupied (1):** >= 5 obstacle hits - obstacle
- **Unknown (-1):** insufficient data

### Step 5: Trajectory Carving

**Problem:** some areas where the robot actually drove may be incorrectly marked as obstacles due to sensor noise or partial overlap.

**Solution:** all cells along the SLAM trajectory are marked as free with a radius of **0.25 m** (5 cells). This guarantees connectivity of the traversable corridor.

```
For each pose (every 5th):
  For each cell within a 0.25 m radius:
    if cell != free -> mark as free
```

**Result:** 15,788 cells carved. Free: 41,147 -> 56,935. Occupied: 53,738 -> 38,170.

### Step 6: Inflation (Obstacle Expansion)

Obstacles are expanded by the **robot radius (0.3 m = 6 cells)** using binary dilation with a circular structuring element. This ensures a safe clearance between the path and obstacles.

```
Inflated cells = binary_dilation(obstacles, circular_kernel(r=6))
```

### Step 7: A* Path Planning

**Algorithm:** A* with 8-connectivity (8-connected grid search).

**Heuristic:** Octile distance - exact for 8-connected grids:
```
h(n) = max(|dr|, |dc|) + (sqrt(2) - 1) * min(|dr|, |dc|)
```

**Movement cost:**
- Cardinal directions (4): cost = 1.0
- Diagonal directions (4): cost = sqrt(2) ~ 1.414

**Traversability:** only cells with value 0 (free) are traversable. Unknown (-1), occupied (1), and inflated (2) are blocked.

**Adaptive inflation:** if no route is found with full inflation (6 cells), the system automatically reduces inflation to [4, 3, 2, 1, 0] until a path is found.

**Snap to free:** if the start/goal falls on an obstacle, the nearest free cell within a 60-cell radius is used.

---

## Results: 3 Navigation Queries

Three routes between different parts of the garden were tested:

| Route | Start -> Goal | Euclidean | A* Path | Ratio | Comment |
|-------|---------------|-----------|---------|-------|---------|
| **A** (red) | Origin -> Far corner | 15.4 m | **17.1 m** | 1.11 | Nearly straight, few obstacles in the way |
| **B** (orange) | Top -> Bottom | 17.0 m | **23.0 m** | 1.35 | Significant detour - obstacles in the middle |
| **C** (purple) | Mid-left -> Mid-right | 11.5 m | **15.6 m** | 1.36 | Detour around walls and bushes |

**Ratio** = A* path / Euclidean distance:
- Ratio = 1.0 -> straight path, no obstacles
- Ratio > 1.0 -> path detours around obstacles
- Routes B and C have ratio ~1.35 -> A* finds detour routes

All 3 routes were found with **full inflation of 0.3 m** (no reduction was needed).

---

## Visualization (3 Panels)

### Panel 1: Occupancy Grid + Trajectories

- **White** - free space
- **Black** - obstacles (occupied)
- **Gray** - unknown space
- **Green line** - GT trajectory (Sim3-aligned to SLAM)
- **Blue line** - ORB-SLAM3 trajectory
- **Colored markers** - start (o) and goal (*) for each route

### Panel 2: Inflated Grid + A* Routes

- **Light blue-gray** - inflation zone (safety margin)
- **Colored lines** - A* paths (red=A, orange=B, purple=C)
- Paths clearly avoid inflated obstacles

### Panel 3: Point Cloud (Top-Down)

- 3D point cloud viewed from above, colored by height (Y)
- **Yellow/green** - ground level (Y ~ +0.2 m)
- **Purple/blue** - objects above camera height (walls, trees)
- The garden structure is clearly visible: walls, bushes, pathways

---

## Technical Details

### ORB-SLAM3 Coordinate System

ORB-SLAM3 uses a **camera coordinate system**:
- **X** - right (13.4 m spread)
- **Y** - down (1.6 m spread - vertical axis)
- **Z** - forward (17.7 m spread)

Horizontal plane: **X-Z**. Vertical axis: **Y** (positive = down).

### Sim3 Alignment GT -> SLAM

GT (Leica) uses a different coordinate system (X-Y horizontal, Z vertical). To overlay GT onto the occupancy grid, **Umeyama alignment** (Sim3) is used:

```
GT_in_SLAM = s * R * GT + t
```

Parameters: scale = 1.0207, residual = 0.367 m.

### Why Trajectory Carving Is Necessary

Without trajectory carving, free corridors have gaps:
1. The depth sensor cannot see the ground directly beneath the camera (blind zone)
2. Depth noise at object boundaries creates false obstacles
3. Degenerate geometry when moving along walls

Carving with a radius of 0.25 m (slightly less than the robot width of ~0.5 m) guarantees:
- Corridor connectivity along the traversed path
- A* can always find a route between points on the trajectory

---

## Files

| File | Description |
|------|-------------|
| `occupancy_astar.py` | Main script (single file, no arguments) |
| `occupancy_grid_astar.png` | 3-panel visualization |
| `occupancy_data.npz` | Saved data: occupancy grid, inflated grid, point cloud |

### Running

```bash
python3 datasets/rover/scripts/occupancy_astar.py
```

Execution time: ~7 seconds (backprojection ~4s, grid+A* ~1s, visualization ~2s).
