import math
import numpy as np
import heapq


class OccupancyGrid:
    """
    local occupancy grid centered on robot.
    only tracks NEW obstacles (not pre-mapped trees).
    recenters on robot each update.
    """

    def __init__(self, x_range=(-110, 85), y_range=(-50, 50), resolution=0.5):
        self.res = resolution
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range
        self.nx = int((self.x_max - self.x_min) / self.res)
        self.ny = int((self.y_max - self.y_min) / self.res)
        # 0=unknown, 1=free, 2=occupied
        self.grid = np.zeros((self.ny, self.nx), dtype=np.uint8)
        # confidence: how many times a cell was observed
        self.hits = np.zeros((self.ny, self.nx), dtype=np.int16)

    def world_to_grid(self, x, y):
        gx = int((x - self.x_min) / self.res)
        gy = int((y - self.y_min) / self.res)
        return max(0, min(gx, self.nx-1)), max(0, min(gy, self.ny-1))

    def grid_to_world(self, gx, gy):
        return self.x_min + (gx + 0.5) * self.res, self.y_min + (gy + 0.5) * self.res

    def update_from_depth(self, depth_img, robot_x, robot_y, robot_yaw):
        """project depth image to grid. marks free space along rays, occupied at endpoints."""
        if depth_img is None:
            return

        h, w = depth_img.shape[:2]
        fx = 320.0
        cx = 320.0
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)

        # sample depth image (every 8th pixel, middle rows only)
        for row in range(h//4, 3*h//4, 8):
            for col in range(0, w, 8):
                z = float(depth_img[row, col])
                if z < 0.5 or z > 8.0:
                    continue

                # camera frame to world
                cam_x = (col - cx) * z / fx
                wx = robot_x + z * cos_y - cam_x * sin_y
                wy = robot_y + z * sin_y + cam_x * cos_y

                # mark endpoint as occupied (if above ground level)
                # skip ground detections: row > 300 and z < 2 is likely ground
                if row > 300 and z < 2.0:
                    continue

                gx, gy = self.world_to_grid(wx, wy)
                if 0 <= gx < self.nx and 0 <= gy < self.ny:
                    if z < 6.0:  # only mark close things as occupied
                        self.grid[gy, gx] = 2
                        self.hits[gy, gx] += 1

                # mark free space along ray (every 1m)
                for d in np.arange(0.5, min(z - 0.3, 6.0), 0.5):
                    fx2 = robot_x + d * cos_y - (cam_x * d / z) * sin_y
                    fy2 = robot_y + d * sin_y + (cam_x * d / z) * cos_y
                    fgx, fgy = self.world_to_grid(fx2, fy2)
                    if 0 <= fgx < self.nx and 0 <= fgy < self.ny:
                        if self.grid[fgy, fgx] != 2:  # don't overwrite occupied
                            self.grid[fgy, fgx] = 1

        # mark robot position as free
        rgx, rgy = self.world_to_grid(robot_x, robot_y)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                nx, ny = rgx+dx, rgy+dy
                if 0 <= nx < self.nx and 0 <= ny < self.ny:
                    self.grid[ny, nx] = 1

    def mark_circle(self, wx, wy, radius):
        """mark a circular area as occupied in world coords"""
        gx_c, gy_c = self.world_to_grid(wx, wy)
        r_cells = int(math.ceil(radius / self.res))
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if dx*dx + dy*dy <= r_cells*r_cells:
                    nx, ny = gx_c + dx, gy_c + dy
                    if 0 <= nx < self.nx and 0 <= ny < self.ny:
                        self.grid[ny, nx] = 2
                        self.hits[ny, nx] = 100  # high confidence

    def is_free(self, gx, gy):
        if 0 <= gx < self.nx and 0 <= gy < self.ny:
            return self.grid[gy, gx] != 2
        return False

    def inflate(self, radius_cells=1):
        """inflate obstacles for robot clearance (husky half-width ~0.33m = 1 cell)"""
        inflated = self.grid.copy()
        occupied = np.argwhere(self.grid == 2)
        for oy, ox in occupied:
            for dx in range(-radius_cells, radius_cells+1):
                for dy in range(-radius_cells, radius_cells+1):
                    nx, ny = ox+dx, oy+dy
                    if 0 <= nx < self.nx and 0 <= ny < self.ny:
                        if inflated[ny, nx] != 2:
                            inflated[ny, nx] = 2
        return inflated


def astar(grid, start_xy, goal_xy, occ_grid):
    """A* on occupancy grid. returns list of (world_x, world_y) waypoints."""
    sx, sy = occ_grid.world_to_grid(*start_xy)
    gx, gy = occ_grid.world_to_grid(*goal_xy)

    # inflate obstacles for robot clearance
    inflated = occ_grid.inflate(radius_cells=2)

    if inflated[sy, sx] == 2:
        # start is in obstacle, find nearest free cell
        for r in range(1, 10):
            found = False
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    nx, ny = sx+dx, sy+dy
                    if 0 <= nx < occ_grid.nx and 0 <= ny < occ_grid.ny:
                        if inflated[ny, nx] != 2:
                            sx, sy = nx, ny
                            found = True
                            break
                if found: break
            if found: break

    if inflated[gy, gx] == 2:
        for r in range(1, 10):
            found = False
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    nx, ny = gx+dx, gy+dy
                    if 0 <= nx < occ_grid.nx and 0 <= ny < occ_grid.ny:
                        if inflated[ny, nx] != 2:
                            gx, gy = nx, ny
                            found = True
                            break
                if found: break
            if found: break

    # A*
    open_set = [(0, sx, sy)]
    came_from = {}
    g_score = {(sx, sy): 0}
    closed = set()

    while open_set:
        _, cx, cy = heapq.heappop(open_set)
        if (cx, cy) in closed:
            continue
        closed.add((cx, cy))

        if abs(cx - gx) <= 1 and abs(cy - gy) <= 1:
            # reconstruct path
            path = []
            node = (cx, cy)
            while node in came_from:
                wx, wy = occ_grid.grid_to_world(*node)
                path.append((wx, wy))
                node = came_from[node]
            path.reverse()
            # thin path: keep every 4th point
            thinned = path[::4]
            if thinned and thinned[-1] != path[-1]:
                thinned.append(path[-1])
            # add goal
            thinned.append(goal_xy)
            return thinned

        # 8-connected neighbors
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = cx+dx, cy+dy
            if 0 <= nx < occ_grid.nx and 0 <= ny < occ_grid.ny:
                if inflated[ny, nx] == 2:
                    continue
                if (nx, ny) in closed:
                    continue
                cost = 1.0 if dx == 0 or dy == 0 else 1.414
                new_g = g_score[(cx, cy)] + cost
                if new_g < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = new_g
                    h = math.hypot(nx - gx, ny - gy)
                    heapq.heappush(open_set, (new_g + h, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)

    # no path found, return direct line
    return [goal_xy]
