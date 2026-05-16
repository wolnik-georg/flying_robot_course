#!/usr/bin/env python3
"""Sanity check: print mocap poses from /poses to verify topic, data, and values.

Run with ROS2 sourced while motion_capture_tracking node is running:
  python3 scripts/mocap_check.py

Press Ctrl+C to stop.
"""

import math
import rclpy
from rclpy.node import Node
from motion_capture_tracking_interfaces.msg import NamedPoseArray

DRONE_NAME = "cf231"

class MocapCheck(Node):
    def __init__(self):
        super().__init__("mocap_check")
        self.create_subscription(NamedPoseArray, "/poses", self._cb, 10)
        self.count = 0
        self.get_logger().info(f"Listening on /poses, looking for '{DRONE_NAME}' ...")
        self.get_logger().info("Will print all seen rigid body names on first message.")

    def _cb(self, msg: NamedPoseArray):
        self.count += 1

        # On first message, print all visible rigid body names
        if self.count == 1:
            names = [p.name for p in msg.poses]
            print(f"\n[first msg] Visible rigid bodies: {names}\n")

        # Find our drone
        for pose in msg.poses:
            if pose.name != DRONE_NAME:
                continue

            p = pose.pose.position
            q = pose.pose.orientation

            if math.isnan(p.x):
                print(f"[msg {self.count:4d}] {DRONE_NAME}: LOST (NaN) — check markers")
                return

            print(
                f"[msg {self.count:4d}] {DRONE_NAME}: "
                f"pos=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) m  "
                f"quat=({q.x:+.3f}, {q.y:+.3f}, {q.z:+.3f}, {q.w:+.3f})"
            )
            return

        # Drone not in this message
        if self.count % 50 == 0:
            names = [p.name for p in msg.poses]
            print(f"[msg {self.count:4d}] '{DRONE_NAME}' not found. Visible: {names}")

def main():
    rclpy.init()
    try:
        rclpy.spin(MocapCheck())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
