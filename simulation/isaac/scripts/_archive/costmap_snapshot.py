#!/usr/bin/env python3
"""
Costmap snapshot logger for exp 33.

Subscribes to /global_costmap/costmap and /local_costmap/costmap.
Tracks robot pose via TF. When the robot reaches max_x near -10 and
then regresses by > 1.0m (first sign of Nav2 planner dead-end),
dumps the current costmaps around the robot to /tmp/ for analysis.

usage:
  source /opt/ros/jazzy/setup.bash
  python3 costmap_snapshot.py
"""
import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


# SPAWN_X = -95.0  # south route default, route 04-09 override this
WINDOW_M = 10.0    # half-window in meters around robot for snapshot
# TRIGGER_MIN_X is in the frame published by the TF relay. In SLAM-frame
# mode this is SLAM forward coordinate - exp 30 stuck zone is ~SLAM x=85.
TRIGGER_MIN_X = 70.0
REGRESSION_DELTA = 1.0  # trigger if robot regressed by this much from max_x
POSE_POLL_HZ = 5.0


class CostmapSnap(Node):
    def __init__(self):
        super().__init__('costmap_snapshot')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_msg = None
        self.local_msg = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._global_cb, 1)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._local_cb, 1)

        self.max_x_seen = -1e9
        self.saved = False
        self.start_time = time.time()
        self.last_robot_pose = None

        self.create_timer(1.0 / POSE_POLL_HZ, self.tick)
        self.get_logger().info('costmap snapshot logger started')

    def _global_cb(self, msg):
        self.global_msg = msg

    def _local_cb(self, msg):
        self.local_msg = msg

    def _get_robot(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None, None

    def tick(self):
        rx, ry = self._get_robot()
        if rx is None:
            return
        self.last_robot_pose = (rx, ry)
        if rx > self.max_x_seen:
            self.max_x_seen = rx

        elapsed = time.time() - self.start_time
        if elapsed % 10 < 0.25:
            self.get_logger().info(
                f'pose=({rx:.1f},{ry:.1f}) max_x={self.max_x_seen:.1f} '
                f'saved={self.saved}')

        if self.saved:
            return

        # Trigger: robot reached x > TRIGGER_MIN_X AND regressed by at
        # least REGRESSION_DELTA. Also require map data present.
        if self.max_x_seen < TRIGGER_MIN_X:
            return
        if rx > self.max_x_seen - REGRESSION_DELTA:
            return
        if self.global_msg is None or self.local_msg is None:
            self.get_logger().warn('regression detected but no costmap msgs yet')
            return

        self._save_snapshot('first_regression', rx, ry)
        self.saved = True

    def _save_snapshot(self, tag, rx, ry):
        for name, msg in (('global', self.global_msg),
                          ('local', self.local_msg)):
            grid = np.array(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width)
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            cx = int((rx - ox) / res)
            cy = int((ry - oy) / res)
            half = int(WINDOW_M / res)
            x0 = max(0, cx - half); x1 = min(grid.shape[1], cx + half)
            y0 = max(0, cy - half); y1 = min(grid.shape[0], cy + half)
            snippet = grid[y0:y1, x0:x1]

            lethal = int(np.sum(snippet >= 253))
            inscribed = int(np.sum((snippet >= 99) & (snippet < 253)))
            inflated = int(np.sum((snippet > 0) & (snippet < 99)))
            free = int(np.sum(snippet == 0))
            unknown = int(np.sum(snippet < 0))

            npy_path = f'/tmp/costmap_{name}_{tag}.npy'
            json_path = f'/tmp/costmap_{name}_{tag}.json'
            np.save(npy_path, snippet)
            with open(json_path, 'w') as f:
                json.dump({
                    'robot_x': float(rx),
                    'robot_y': float(ry),
                    'max_x_seen': float(self.max_x_seen),
                    'origin_x_world': float(ox + x0 * res),
                    'origin_y_world': float(oy + y0 * res),
                    'resolution': float(res),
                    'shape': list(snippet.shape),
                    'cells': {
                        'lethal': lethal,
                        'inscribed': inscribed,
                        'inflated': inflated,
                        'free': free,
                        'unknown': unknown,
                    },
                }, f, indent=2)
            self.get_logger().warn(
                f'[{name}] SNAPSHOT @ ({rx:.1f},{ry:.1f}) '
                f'max_x={self.max_x_seen:.1f}: '
                f'{snippet.shape} cells, lethal={lethal} '
                f'inscribed={inscribed} inflated={inflated} '
                f'free={free} unknown={unknown}')


def main():
    rclpy.init()
    node = CostmapSnap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
