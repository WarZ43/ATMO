#!/usr/bin/env python3
"""Publish a synthetic /vrpn_mocap/<body>/pose at the Motive rate.

Stands in for the OptiTrack chain when the arena is unavailable, so the
LINK acceptance test can run without it: this stresses exactly what the
real stream will -- 120 Hz of PoseStamped over DDS across the WiFi -- and
the far side measures it with the same tools the real session uses
(mocap_bridge rate/gap reporting, hardware_optitrack_check --mode rate).

The pose is a slow horizontal circle with a gentle z bob, so the bridge's
differentiated velocity is non-trivial and sign-checkable. This validates
TRANSPORT, not frames: the C1/C2 sign gates still need the real rig.

    ./scripts/fake_vrpn_publisher.py                # M4, 120 Hz, z-up
    ./scripts/fake_vrpn_publisher.py --body M4 --rate-hz 120
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class FakeVrpn(Node):
    def __init__(self, body: str, rate_hz: float):
        super().__init__("fake_vrpn_publisher")
        self.publisher = self.create_publisher(
            PoseStamped, f"/vrpn_mocap/{body}/pose", qos_profile_sensor_data
        )
        self.rate_hz = rate_hz
        self.count = 0
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f"Publishing synthetic /vrpn_mocap/{body}/pose at {rate_hz:.0f} Hz"
        )

    def tick(self):
        t = self.count / self.rate_hz
        self.count += 1
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = 0.5 * math.cos(0.2 * t)
        msg.pose.position.y = 0.5 * math.sin(0.2 * t)
        msg.pose.position.z = 0.3 + 0.05 * math.sin(0.5 * t)
        yaw = 0.1 * math.sin(0.3 * t)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        self.publisher.publish(msg)
        if self.count % (int(self.rate_hz) * 10) == 0:
            self.get_logger().info(f"{self.count} poses published")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--body", default="M4")
    parser.add_argument("--rate-hz", type=float, default=120.0)
    args = parser.parse_args()
    rclpy.init()
    node = FakeVrpn(args.body, args.rate_hz)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
