#!/usr/bin/env python3
"""
Follow recorded teach-run trajectory via Nav2 sequential goals.
On failure: backup, clear costmap, skip to next waypoint.
"""
import argparse
import json
import math
import os
import time
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from action_msgs.msg import GoalStatus


class TrajectoryFollower(Node):
    def __init__(self, waypoints_file, direction="outbound", spacing=4.0):
        super().__init__("trajectory_follower")
        self.direction = direction

        with open(waypoints_file) as f:
            all_anchors = json.load(f)
        max_x = max(range(len(all_anchors)), key=lambda i: all_anchors[i]["x"])
        outbound = all_anchors[: max_x + 1]

        self.waypoints = []
        last = None
        for a in outbound:
            if last is None or math.hypot(a["x"] - last[0], a["y"] - last[1]) > spacing:
                self.waypoints.append(a)
                last = (a['x'], a["y"])
        if math.hypot(outbound[-1]["x"] - last[0], outbound[-1]["y"] - last[1]) > 1.0:
            self.waypoints.append(outbound[-1])

        self.get_logger().info(f"Trajectory: {len(self.waypoints)} waypoints, "
                               f"({self.waypoints[0]['x']:.0f},{self.waypoints[0]['y']:.0f}) -> "
                               f"({self.waypoints[-1]['x']:.0f},{self.waypoints[-1]['y']:.0f})")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Diagnostics
        self.latest_global_cm = None
        self.latest_local_cm = None
        self.latest_plan = None
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._gcm_cb, 1)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self._lcm_cb, 1)
        self.create_subscription(Path, "/plan", self._plan_cb, 1)
        self.debug_dir = os.environ.get("DEBUG_DIR", "/tmp/nav2_debug")
        os.makedirs(self.debug_dir, exist_ok=True)

        self.get_logger().info("Waiting for Nav2...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready.")

        self.reached = 0
        self.skipped = 0
        self.start_time = time.time()

        self.MAX_ATTEMPTS = 3 if direction == "return" else 2
        self.GOAL_TIMEOUT = 50.0 if direction == "return" else 40.0
        self.BACKUP_DURATION = 1.5
        self.BACKUP_SPEED = -0.2

    def _gcm_cb(self, msg):
        self.latest_global_cm = msg

    def _lcm_cb(self, msg):
        self.latest_local_cm = msg

    def _plan_cb(self, msg):
        self.latest_plan = msg

    def _save_costmap_debug(self, wp_idx, attempt, robot_xy, wp_xy):
        """Save global+local costmap PNGs + stats around robot."""
        try:
            import cv2
        except ImportError:
            cv2 = None

        for tag, cm in [("global", self.latest_global_cm),
                        ("local", self.latest_local_cm)]:
            if cm is None:
                self.get_logger().warn(f"  [diag] no {tag} costmap yet")
                continue
            w, h = cm.info.width, cm.info.height
            res = cm.info.resolution
            ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
            grid = np.array(cm.data, dtype=np.int8).reshape(h, w)

            if cv2:
                img = np.full((h, w, 3), 200, dtype=np.uint8)
                img[grid == -1] = [100, 100, 100]
                img[(grid > 0) & (grid < 99)] = [0, 255, 255]
                img[(grid >= 99) & (grid < 253)] = [0, 165, 255]
                img[grid >= 253] = [0, 0, 255]
                rx = int((robot_xy[0] - ox) / res)
                ry = int((robot_xy[1] - oy) / res)
                if 0 <= rx < w and 0 <= ry < h:
                    cv2.circle(img, (rx, ry), 4, (0, 255, 0), -1)
                wx = int((wp_xy[0] - ox) / res)
                wy = int((wp_xy[1] - oy) / res)
                if 0 <= wx < w and 0 <= wy < h:
                    cv2.circle(img, (wx, wy), 4, (255, 0, 255), -1)
                # plan polyline
                if self.latest_plan and self.latest_plan.poses:
                    pts = []
                    for p in self.latest_plan.poses:
                        pxs = int((p.pose.position.x - ox) / res)
                        pys = int((p.pose.position.y - oy) / res)
                        pts.append([pxs, pys])
                    pts = np.array(pts, np.int32).reshape(-1, 1, 2)
                    cv2.polylines(img, [pts], False, (255, 255, 255), 1)
                img = cv2.flip(img, 0)
                fname = f"{self.debug_dir}/wp{wp_idx:02d}_att{attempt}_{tag}.png"
                cv2.imwrite(fname, img)

            r_cells = int(4.0 / res)
            rx = int((robot_xy[0] - ox) / res)
            ry = int((robot_xy[1] - oy) / res)
            x1, y1 = max(0, rx - r_cells), max(0, ry - r_cells)
            x2, y2 = min(w, rx + r_cells), min(h, ry + r_cells)
            local = grid[y1:y2, x1:x2]
            tot = max(local.size, 1)
            free = int(np.sum(local == 0))
            leth = int(np.sum(local >= 253))
            infl = int(np.sum((local > 0) & (local < 253)))
            unk = int(np.sum(local == -1))
            # print(f"DEBUG state={state} pose={pose}")
            self.get_logger().warn(
                f"  [{tag} cm ±4m] free={free*100//tot}% lethal={leth*100//tot}% "
                f"inflated={infl*100//tot}% unknown={unk*100//tot}%"
            )

        if self.latest_plan is None or not self.latest_plan.poses:
            self.get_logger().warn("  [diag] /plan empty - PLANNER FAILED")
        else:
            last = self.latest_plan.poses[-1].pose.position
            # print("DEBUG: isaac sim step")
            self.get_logger().warn(
                f"  [diag] /plan has {len(self.latest_plan.poses)} poses, "
                f"ends at ({last.x:.1f},{last.y:.1f}) "
                f"(goal: {wp_xy[0]:.1f},{wp_xy[1]:.1f})"
            )

    def _get_robot_xy(self):
        try:
            with open("/tmp/isaac_pose.txt") as f:
                parts = f.readline().split()
            return float(parts[0]), float(parts[1])
        except Exception:
            return None, None

    def run(self):
        i = 0
        while i < len(self.waypoints):
            wp = self.waypoints[i]

            # Skip waypoints behind the robot or very close
            rx, ry = self._get_robot_xy()
            if rx is not None:
                dx = wp["x"] - rx
                dist = math.hypot(wp["x"] - rx, wp["y"] - ry)
                # For outbound (positive x), WP behind = dx < -2
                # For return (negative x), WP behind = dx > 2
                behind = dx < -2.0 if self.direction == "outbound" else dx > 2.0
                if behind:
                    self.get_logger().info(f"  WP {i} behind robot ({wp['x']:.0f} vs {rx:.0f}), skip")
                    self.skipped += 1
                    i += 1
                    continue
                if dist < 2.5:
                    self.get_logger().info(f"  WP {i} within {dist:.1f}m, counting as reached")
                    self.reached += 1
                    i += 1
                    continue

            self.get_logger().info(
                f"WP {i}/{len(self.waypoints)-1}: ({wp['x']:.1f}, {wp['y']:.1f})"
            )

            success = False
            for attempt in range(self.MAX_ATTEMPTS):
                result = self._send_goal(wp)
                if result:
                    self.reached += 1
                    self.get_logger().info(f"  WP {i} REACHED")
                    success = True
                    break

                self.get_logger().warn(
                    f"  WP {i} failed (attempt {attempt + 1}/{self.MAX_ATTEMPTS})"
                )
                rx, ry = self._get_robot_xy()
                if rx is not None:
                    self._save_costmap_debug(
                        i, attempt + 1, (rx, ry), (wp["x"], wp["y"])
                    )

                if attempt < self.MAX_ATTEMPTS - 1:
                    # v11: NO backup - accumulates drift over many WP failures.
                    # Just clear costmap and retry.
                    self._clear_local_only()
                    time.sleep(0.3)

            if not success:
                self.skipped += 1
                self.get_logger().warn(f"  WP {i} SKIPPED after {self.MAX_ATTEMPTS} attempts")

            i += 1

        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"RESULT:\n"
            f"  Reached: {self.reached}/{len(self.waypoints)}\n"
            f"  Skipped: {self.skipped}\n"
            f"  Duration: {elapsed:.0f}s\n"
            f"{'='*50}"
        )

    def _send_goal(self, wp):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp["x"])
        goal.pose.pose.position.y = float(wp["y"])
        yaw = float(wp.get("yaw", 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return False
        gh = future.result()
        if not gh.accepted:
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.GOAL_TIMEOUT
        )

        if result_future.result() is not None:
            return result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        else:
            gh.cancel_goal_async()
            time.sleep(0.5)
            return False

    def _backup(self):
        self.get_logger().info("  backing up...")
        twist = Twist()
        twist.linear.x = self.BACKUP_SPEED
        t0 = time.time()
        while time.time() - t0 < self.BACKUP_DURATION:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def _clear_local_only(self):
        try:
            subprocess.run(
                ["ros2", "service", "call",
                 "/local_costmap/clear_entirely_local_costmap",
                 "nav2_msgs/srv/ClearEntireCostmap", "{}"],
                timeout=3, capture_output=True,
            )
        except Exception:
            pass

    def _clear_costmaps(self):
        try:
            subprocess.run(
                ["ros2", "service", "call",
                 "/global_costmap/clear_entirely_global_costmap",
                 "nav2_msgs/srv/ClearEntireCostmap", "{}"],
                timeout=3, capture_output=True,
            )
            subprocess.run(
                ["ros2", "service", "call",
                 "/local_costmap/clear_entirely_local_costmap",
                 "nav2_msgs/srv/ClearEntireCostmap", "{}"],
                timeout=3, capture_output=True,
            )
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--anchors",
        default="/workspace/simulation/isaac/route_memory/road/anchors.json",
    )
    parser.add_argument(
        "--direction", default="outbound", choices=["outbound", "return"],
    )
    parser.add_argument(
        "--spacing", type=float, default=4.0,
        help="Waypoint spacing (m). Default 4.0 for road; use 2.0 for dense forest.",
    )
    args = parser.parse_args()

    rclpy.init()
    follower = TrajectoryFollower(args.anchors, direction=args.direction, spacing=args.spacing)
    if args.direction == "return":
        follower.waypoints = list(reversed(follower.waypoints))
        follower.get_logger().info(
            f"RETURN mode: {len(follower.waypoints)} waypoints reversed"
        )
    try:
        follower.run()
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
