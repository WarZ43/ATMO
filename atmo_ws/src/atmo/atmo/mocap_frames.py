"""Frame conversions for the OptiTrack bridge, with no ROS dependency.

Split out from `mocap_bridge.py` so the arithmetic can be tested on any machine
-- these are the functions where a sign error becomes a crash, and they should
not require a robot, a mocap rig or a ROS install to check.

Quaternions are (w, x, y, z) throughout.

FRAME CONVENTIONS ARE ASSUMPTIONS UNTIL CONFIRMED BY MOTION. Nothing in this
module knows which convention your Motive rig streams; it only implements each
one faithfully. `scripts/hardware_optitrack_check.py` is what decides.
"""

import numpy as np


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.asarray(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    )


def quat_conjugate(q):
    return np.asarray((q[0], -q[1], -q[2], -q[3]))


def rotate_by_quat_inverse(q, vector):
    """Rotate a world vector into the body frame."""
    pure = np.concatenate(((0.0,), np.asarray(vector, dtype=float)))
    return quat_multiply(quat_conjugate(q), quat_multiply(pure, q))[1:]


def to_z_up(position, quaternion, source_frame):
    """Bring the streamed Motive pose into the policy's z-up world."""
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if source_frame == "z_up":
        return position, quaternion
    if source_frame != "y_up":
        raise ValueError("source_frame must be y_up or z_up, got %r" % (source_frame,))
    # Motive's default y-up stream: world = (x, -z, y), right-handed.
    remapped = np.asarray((position[0], -position[2], position[1]))
    w, x, y, z = quaternion
    return remapped, np.asarray((w, x, -z, y))


def apply_mount_yaw(quaternion, yaw_rad):
    """Rotate the rigid body's own axes by `yaw_rad` about its z.

    For when the Motive rigid body is defined at a different heading from the
    flight controller and the rotor numbering, which is a mounting fact that
    cannot always be corrected on the vehicle. The body frame is RELABELLED:

        R_world_from_corrected = R_world_from_streamed . Rz(yaw)

    so the streamed world position is untouched (relabelling the axes does not
    move the origin), and any body-frame vector derived from the corrected
    quaternion afterwards -- the twist this bridge differentiates, for instance
    -- comes out in the corrected frame for free.

    Measured on ATMO 2026-08-20 (2026-08-20 frame analysis, CLAIM 1): the streamed
    body frame sits 178.6-179.9 deg from the FC's across three flights, i.e.
    the rig is mounted 180 deg out, and nothing downstream removed it. That is
    the roll inversion that departed log_417.
    """
    half = 0.5 * float(yaw_rad)
    return quat_multiply(
        np.asarray(quaternion, dtype=float),
        np.asarray((np.cos(half), 0.0, 0.0, np.sin(half))),
    )


def z_up_to_ned_position(position):
    """north = x, east = -y, down = -z."""
    return (float(position[0]), float(-position[1]), float(-position[2]))


def z_up_to_ned_velocity(velocity):
    """Same axis mapping as the position."""
    return (float(velocity[0]), float(-velocity[1]), float(-velocity[2]))


def px4_quaternion_composed(quaternion):
    """A z-up world quaternion in PX4's NED convention."""
    w, x, y, z = quaternion
    return (float(w), float(x), float(-y), float(-z))


def px4_quaternion_atmo_legacy(raw_quaternion):
    """The mapping relay_mocap.py used, applied to the RAW streamed quaternion.

    Kept bit-for-bit because it is the one that has flown on this airframe.
    Note it is NOT the same rotation as composing y_up -> z_up -> NED; see the
    module docstring of mocap_bridge.py. Which is correct is a measurement.
    """
    w, x, y, z = raw_quaternion
    return (float(w), float(-z), float(x), float(-y))


def px4_position_atmo_legacy(raw_position):
    """The mapping relay_mocap.py used, applied to the RAW streamed position."""
    return (float(raw_position[0]), float(raw_position[2]), float(-raw_position[1]))
