"""PX4 uXRCE-DDS topic names, which are not stable across firmware versions.

PX4 1.16 introduced message versioning on the DDS interface: some topics gained
a `_v1` suffix, and which ones did is not guessable -- it is per-message, not a
blanket rename. Measured on this vehicle (Cube Orange, PX4 v1.17.0, git
8874d533, built 2025-11-26), `vehicle_status` is versioned while
`vehicle_odometry`, `input_rc` and `actuator_motors` are not.

A wrong topic name here does not crash anything. The subscription simply never
receives, and the node runs looking healthy while flying blind on that signal.
That is why the names live in one place with an explicit inventory behind them,
and why `warn_missing()` exists: a topic that is not on the graph should say so
at startup rather than at the worst moment.

Every name is overridable by environment variable, so a firmware change is a
config edit rather than a code change. See the README for the full
measured inventory and how to re-take it.
"""

import os

# Logical name -> (default topic for PX4 1.17, environment override).
_TOPICS = {
    # --- companion publishes, PX4 subscribes -------------------------------
    "actuator_motors": "/fmu/in/actuator_motors",
    "offboard_control_mode": "/fmu/in/offboard_control_mode",
    "vehicle_command": "/fmu/in/vehicle_command",
    "vehicle_visual_odometry": "/fmu/in/vehicle_visual_odometry",
    "trajectory_setpoint": "/fmu/in/trajectory_setpoint",
    "vehicle_thrust_setpoint": "/fmu/in/vehicle_thrust_setpoint",
    # NOT in this firmware's dds_topics.yaml -- see the README. It
    # still works node-to-node (TiltControllerBase publishes it, the RL
    # runtime subscribes), but PX4 itself never receives the tilt angle.
    "tilt_angle": "/fmu/in/tilt_angle",
    # --- PX4 publishes, companion subscribes -------------------------------
    "input_rc": "/fmu/out/input_rc",
    "vehicle_odometry": "/fmu/out/vehicle_odometry",
    "vehicle_command_ack": "/fmu/out/vehicle_command_ack",
    "vehicle_control_mode": "/fmu/out/vehicle_control_mode",
    "failsafe_flags": "/fmu/out/failsafe_flags",
    "estimator_status_flags": "/fmu/out/estimator_status_flags",
    # Versioned on 1.16+. Plain `/fmu/out/vehicle_status` is PX4 <= 1.15 and
    # silently never receives on this firmware.
    "vehicle_status": "/fmu/out/vehicle_status_v1",
    "battery_status": "/fmu/out/battery_status_v1",
    "vehicle_local_position": "/fmu/out/vehicle_local_position_v1",
    "home_position": "/fmu/out/home_position_v1",
}

# Names to try when the configured one is absent, so the same code runs against
# an older flight controller without an edit.
_FALLBACKS = {
    "vehicle_status": ("/fmu/out/vehicle_status",),
    "battery_status": ("/fmu/out/battery_status",),
    "vehicle_local_position": ("/fmu/out/vehicle_local_position",),
    "home_position": ("/fmu/out/home_position",),
}


def _env_name(logical):
    return "ATMO_PX4_TOPIC_" + logical.upper()


def topic(logical):
    """The topic name to use for a logical PX4 signal."""
    if logical not in _TOPICS:
        raise KeyError("unknown PX4 topic %r" % (logical,))
    return os.getenv(_env_name(logical), _TOPICS[logical])


def candidates(logical):
    """Every name worth trying for this signal, preferred first."""
    return (topic(logical),) + tuple(_FALLBACKS.get(logical, ()))


def resolve(node, logical):
    """Pick whichever candidate is actually on the graph right now.

    Falls back to the preferred name when none is present -- the publisher may
    simply not have started yet, and subscribing to a not-yet-existent topic is
    normal in ROS. `warn_missing` is what reports that case.
    """
    try:
        available = {name for name, _ in node.get_topic_names_and_types()}
    except Exception:
        return topic(logical)
    for name in candidates(logical):
        if name in available:
            return name
    return topic(logical)


def warn_missing(node, logicals):
    """Log the signals that are not on the graph, with what was expected.

    Called at startup. It is advisory, not fatal: PX4 creates a topic when
    something first publishes it, so an absence here can be a timing artifact.
    A name that stays absent is a version mismatch.
    """
    try:
        available = {name for name, _ in node.get_topic_names_and_types()}
    except Exception:
        return []
    missing = []
    for logical in logicals:
        if not any(name in available for name in candidates(logical)):
            missing.append((logical, topic(logical)))
    if missing:
        node.get_logger().warn(
            "PX4 topics not on the graph: "
            + ", ".join("%s (expected %s)" % pair for pair in missing)
            + ". If these stay absent, the firmware names them differently -- "
            "see the README and override with ATMO_PX4_TOPIC_<NAME>."
        )
    return missing
