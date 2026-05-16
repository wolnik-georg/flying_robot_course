#!/usr/bin/env python3
"""Forward mocap pose for cf231 from motion_capture_tracking to Rust over UDP.

Launch the motion_capture_tracking node separately (NOT the full CS2 server,
which would conflict with the Rust script's radio connection):

  ros2 run motion_capture_tracking motion_capture_tracking_node \
    --ros-args --params-file <path_to_crazyswarm2>/crazyflie/config/motion_capture.yaml

Then run this script alongside the Rust onboard binary:

  python3 scripts/mocap_bridge.py

Subscribes to /poses (NamedPoseArray, all rigid bodies) and filters for the
drone named DRONE_NAME. Sends 7 × float32 little-endian [x, y, z, qx, qy, qz, qw]
to UDP localhost:9872 where the Rust script reads and calls send_external_pose().
"""

import struct
import socket
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from motion_capture_tracking_interfaces.msg import NamedPoseArray

DRONE_NAME = "cf231"
UDP_HOST   = "141.23.175.125"  # laptop IP on lab network
UDP_PORT   = 9872

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class MocapBridge(Node):
    def __init__(self):
        super().__init__("mocap_bridge")
        self.create_subscription(NamedPoseArray, "/poses", self._cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"Bridging /poses[{DRONE_NAME}] → UDP {UDP_HOST}:{UDP_PORT}"
        )

    def _cb(self, msg: NamedPoseArray):
        for pose in msg.poses:
            if pose.name != DRONE_NAME:
                continue
            p = pose.pose.position
            q = pose.pose.orientation
            if math.isnan(p.x) or math.isnan(q.x):
                print(f"[mocap] NaN — markers lost, dropping")
                return
            data = struct.pack("<7f", p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            sock.sendto(data, (UDP_HOST, UDP_PORT))
            print(f"[mocap] sent pos=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) to {UDP_HOST}:{UDP_PORT}")
            return  # only one entry per name

def main():
    rclpy.init()
    rclpy.spin(MocapBridge())

if __name__ == "__main__":
    main()
