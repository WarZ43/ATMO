"""OptiTrack bridge: VRPN pose in, policy odometry and PX4 vision out.

Supersedes `relay_mocap.py`, which is kept for reference. Three things this
adds, each of which cost time on the m4-direct-rl bring-up:

1. **Velocity.** The old relay published `velocity = [0, 0, 0]` with
   `velocity_frame` set to NED. PX4 does not read that as "no velocity
   supplied" -- it reads it as a measurement that the vehicle is stationary,
   and fuses it. The correct way to decline to provide velocity is NaN, which
   is what this node sends unless `publish_velocity` is set. When it is set,
   the velocity is a real filtered derivative rather than a constant.
2. **A policy odometry topic.** `nav_msgs/Odometry` on
   `/atmo/groundtruth_odom` with pose in the z-up world and twist in the BODY
   frame. Anything that wants mocap without going through the EKF reads this.
3. **Rate and staleness monitoring.** A mocap stream that degrades rather than
   stops is the failure mode that looks like bad control. This node reports
   measured rate, gaps, and dropouts, so Stage C has numbers to gate on.

FRAME CONVENTIONS ARE ASSUMPTIONS UNTIL CONFIRMED BY MOTION. Motive can stream
either y-up (its default) or z-up (configurable, and what the m4 rig used).
Choose with `source_frame`. Verify with `hardware_optitrack_check.py` and the
Stage C gates in `docs/optitrack_bringup.md` -- move the vehicle by hand in each
axis and confirm the sign, every session. Never trust the label.

Note on the PX4 quaternion: `relay_mocap.py` mapped the body quaternion to NED
as (w, -z, x, -y), while the m4-direct-rl bridge composes y-up -> z-up -> NED
and arrives at (w, x, z, -y). These are NOT the same rotation. This node keeps
the ATMO mapping as the default (`px4_quaternion=atmo_legacy`) because that is
the one that has flown on this airframe, and exposes the m4 composition as
`px4_quaternion=composed`. Which is correct depends on how the rigid body was
defined in Motive, so it is a measurement, not a preference. Stage C3 settles
it; do not fly a changed value that has not been confirmed by motion.

Run:

    ros2 run atmo mocap_bridge --ros-args \\
        -p body:=m4_base -p source_frame:=y_up -p px4_relay:=true
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from atmo.mocap_frames import (
    px4_position_atmo_legacy,
    apply_mount_yaw,
    px4_quaternion_atmo_legacy,
    px4_quaternion_composed,
    quat_conjugate,
    quat_multiply,
    rotate_by_quat_inverse,
    to_z_up,
    z_up_to_ned_position,
    z_up_to_ned_velocity,
)

NAN = float("nan")


class MocapBridge(Node):
    def __init__(self):
        super().__init__("atmo_mocap_bridge")

        self.declare_parameter("body", "m4_base")
        self.declare_parameter("topic", "")
        self.declare_parameter("source_frame", "y_up")
        # Heading of the Motive rigid body relative to the flight controller and
        # the rotor numbering. 180 is this vehicle's measured mounting; see
        # apply_mount_yaw. Set 0 only if the rigid body is redefined to match.
        self.declare_parameter("mount_yaw_deg", 180.0)
        self.declare_parameter("odom_topic", "/atmo/groundtruth_odom")
        self.declare_parameter("px4_relay", True)
        self.declare_parameter("px4_quaternion", "atmo_legacy")
        self.declare_parameter("publish_velocity", False)
        self.declare_parameter("expected_rate_hz", 120.0)
        self.declare_parameter("filter_hz", 8.0)
        self.declare_parameter("report_period_s", 5.0)
        self.declare_parameter("stale_after_s", 0.25)

        self.body = self.get_parameter("body").value
        topic = self.get_parameter("topic").value
        self.topic = topic if topic else "/vrpn_mocap/%s/pose" % self.body
        self.source_frame = self.get_parameter("source_frame").value
        self.mount_yaw_rad = math.radians(float(self.get_parameter("mount_yaw_deg").value))
        if self.source_frame not in ("y_up", "z_up"):
            raise ValueError("source_frame must be y_up or z_up")
        self.px4_quaternion_mode = self.get_parameter("px4_quaternion").value
        if self.px4_quaternion_mode not in ("atmo_legacy", "composed"):
            raise ValueError("px4_quaternion must be atmo_legacy or composed")
        self.publish_velocity = bool(self.get_parameter("publish_velocity").value)
        self.expected_rate_hz = float(self.get_parameter("expected_rate_hz").value)
        self.stale_after_s = float(self.get_parameter("stale_after_s").value)

        filter_hz = float(self.get_parameter("filter_hz").value)
        self.alpha = float(
            np.clip(
                2.0 * math.pi * filter_hz / max(self.expected_rate_hz, 1.0), 0.0, 1.0
            )
        )

        self.last_stamp = None
        self.last_position = np.zeros(3)
        self.last_quaternion = np.asarray((1.0, 0.0, 0.0, 0.0))
        self.linear_world = np.zeros(3)
        self.angular_body = np.zeros(3)

        # Stream health, reported rather than inferred.
        self.received = 0
        self.rejected = 0
        self.gaps = 0
        self.worst_gap_s = 0.0
        self.window_start = None
        self.window_count = 0
        self.last_receive_wall = None
        self.warned_stale = False

        self.odom_publisher = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, qos_profile_sensor_data
        )
        self.px4_publisher = None
        if bool(self.get_parameter("px4_relay").value):
            from px4_msgs.msg import VehicleOdometry

            self.VehicleOdometry = VehicleOdometry
            self.px4_publisher = self.create_publisher(
                VehicleOdometry,
                "/fmu/in/vehicle_visual_odometry",
                qos_profile_sensor_data,
            )

        self.create_subscription(
            PoseStamped, self.topic, self._pose_callback, qos_profile_sensor_data
        )
        self.create_timer(
            float(self.get_parameter("report_period_s").value), self._report
        )

        self.get_logger().info(
            "Bridging %s (source_frame=%s, mount_yaw=%.1f deg) -> %s%s"
            % (
                self.topic,
                self.source_frame,
                math.degrees(self.mount_yaw_rad),
                self.get_parameter("odom_topic").value,
                (
                    "; PX4 vision relay ON (quaternion=%s, velocity=%s)"
                    % (
                        self.px4_quaternion_mode,
                        "derived" if self.publish_velocity else "NaN (not supplied)",
                    )
                    if self.px4_publisher is not None
                    else ""
                ),
            )
        )
        self.get_logger().warn(
            "Frame conventions are UNVERIFIED until confirmed by motion. "
            "Run hardware_optitrack_check.py and the Stage C gates before flight."
        )

    # -- callbacks -------------------------------------------------------

    def _pose_callback(self, message):
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        raw_position = np.asarray(
            (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            )
        )
        raw_quaternion = np.asarray(
            (
                message.pose.orientation.w,
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
            )
        )
        if not (
            np.all(np.isfinite(raw_position)) and np.all(np.isfinite(raw_quaternion))
        ):
            self.rejected += 1
            return
        norm = float(np.linalg.norm(raw_quaternion))
        if norm <= 1e-12:
            self.rejected += 1
            return
        raw_quaternion = raw_quaternion / norm

        position, quaternion = to_z_up(
            raw_position, raw_quaternion, self.source_frame
        )
        # Correct the mounting BEFORE the twist is differentiated below, so the
        # body-frame rates come out in the corrected frame with no further work.
        if self.mount_yaw_rad != 0.0:
            quaternion = apply_mount_yaw(quaternion, self.mount_yaw_rad)

        now = self._wall_clock()
        self.received += 1
        self.window_count += 1
        if self.window_start is None:
            self.window_start = now
        if self.last_receive_wall is not None:
            gap = now - self.last_receive_wall
            if gap > self.stale_after_s:
                self.gaps += 1
                self.get_logger().warn("Mocap gap of %.0f ms" % (gap * 1000.0))
            self.worst_gap_s = max(self.worst_gap_s, gap)
        self.last_receive_wall = now
        self.warned_stale = False

        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            if 1e-4 < dt < 0.5:
                raw_linear = (position - self.last_position) / dt
                delta = quat_multiply(
                    quat_conjugate(self.last_quaternion), quaternion
                )
                if delta[0] < 0.0:
                    delta = -delta
                raw_angular = 2.0 * delta[1:] / dt
                self.linear_world += self.alpha * (raw_linear - self.linear_world)
                self.angular_body += self.alpha * (raw_angular - self.angular_body)
        self.last_stamp = stamp
        self.last_position = position
        self.last_quaternion = quaternion

        self._publish_odometry(message, position, quaternion)
        if self.px4_publisher is not None:
            self._publish_px4(position, quaternion, raw_position, raw_quaternion)

    def _publish_odometry(self, message, position, quaternion):
        odom = Odometry()
        odom.header.stamp = message.header.stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.w = float(quaternion[0])
        odom.pose.pose.orientation.x = float(quaternion[1])
        odom.pose.pose.orientation.y = float(quaternion[2])
        odom.pose.pose.orientation.z = float(quaternion[3])
        # Twist in the BODY frame, matching the Gazebo odometry convention the
        # policy runtimes are written against.
        linear_body = rotate_by_quat_inverse(quaternion, self.linear_world)
        odom.twist.twist.linear.x = float(linear_body[0])
        odom.twist.twist.linear.y = float(linear_body[1])
        odom.twist.twist.linear.z = float(linear_body[2])
        odom.twist.twist.angular.x = float(self.angular_body[0])
        odom.twist.twist.angular.y = float(self.angular_body[1])
        odom.twist.twist.angular.z = float(self.angular_body[2])
        self.odom_publisher.publish(odom)

    def _publish_px4(self, position, quaternion, raw_position, raw_quaternion):
        vision = self.VehicleOdometry()
        vision.timestamp = 0
        vision.timestamp_sample = 0
        vision.pose_frame = self.VehicleOdometry.POSE_FRAME_NED
        if self.px4_quaternion_mode == "atmo_legacy":
            vision.position = list(px4_position_atmo_legacy(raw_position))
            vision.q = list(px4_quaternion_atmo_legacy(raw_quaternion))
        else:
            vision.position = list(z_up_to_ned_position(position))
            vision.q = list(px4_quaternion_composed(quaternion))
        if self.publish_velocity:
            vision.velocity_frame = self.VehicleOdometry.VELOCITY_FRAME_NED
            vision.velocity = list(z_up_to_ned_velocity(self.linear_world))
        else:
            # NaN is how PX4 is told a field is not supplied. Zeros are a
            # measurement of stationarity, which is what the old relay sent.
            vision.velocity = [NAN, NAN, NAN]
        vision.angular_velocity = [NAN, NAN, NAN]
        vision.position_variance = [0.0, 0.0, 0.0]
        vision.orientation_variance = [0.0, 0.0, 0.0]
        vision.velocity_variance = [0.0, 0.0, 0.0]
        self.px4_publisher.publish(vision)

    # -- health ----------------------------------------------------------

    def _wall_clock(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _report(self):
        now = self._wall_clock()
        if self.received == 0:
            self.get_logger().warn(
                "No mocap on %s yet. Check that vrpn_mocap is running on the "
                "laptop, that the rigid body is named '%s' and tracked in "
                "Motive, and that both machines share ROS_DOMAIN_ID and RMW."
                % (self.topic, self.body)
            )
            return
        if self.last_receive_wall is not None:
            age = now - self.last_receive_wall
            if age > self.stale_after_s:
                if not self.warned_stale:
                    self.get_logger().error(
                        "Mocap STALE: %.2f s since the last pose" % age
                    )
                    self.warned_stale = True
                return
        if self.window_start is not None and self.window_count > 1:
            span = now - self.window_start
            rate = self.window_count / span if span > 0.0 else 0.0
            message = (
                "mocap %.1f Hz (expected %.0f), %d frames, %d rejected, %d gaps, "
                "worst gap %.0f ms, |v|=%.2f m/s"
                % (
                    rate,
                    self.expected_rate_hz,
                    self.received,
                    self.rejected,
                    self.gaps,
                    self.worst_gap_s * 1000.0,
                    float(np.linalg.norm(self.linear_world)),
                )
            )
            # Two literal call sites, deliberately. rclpy caches the severity
            # per source line, so `level = info-or-warn; level(msg)` raises
            # "Logger severity cannot be changed between calls" the first time
            # the rate crosses the threshold -- killing the bridge precisely
            # when it has something to warn about. Measured 2026-08-17.
            if rate < 0.8 * self.expected_rate_hz:
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)
        self.window_start = now
        self.window_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = MocapBridge()
    # A 120 Hz callback loop stalls ~90 ms roughly every 25 s under CPython's
    # automatic generation-2 collection -- measured on the Jetson 2026-08-17
    # (raw topic clean in the same window, bridge output gapping). Freeze the
    # post-init heap out of the collector and push the full-collection
    # threshold out of reach; gen-0/1 collections stay on and are fast. The
    # steady-state loop allocates only short-lived objects, so nothing
    # accumulates for a full collection to find.
    import gc
    gc.collect()
    gc.freeze()
    gc.set_threshold(700, 10, 1_000_000)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
