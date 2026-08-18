#!/usr/bin/env python3
"""Stage 0: RC channels, gate polarity, and PX4 kill state. Read-only.

THIS SCRIPT CREATES NO PUBLISHERS. It cannot move anything, arm anything, or
command anything. It subscribes and prints. That is what makes it safe to be
the first thing run in a session, before the vehicle has earned any trust.

Run it before every session. On the White M4 this stage found two things that
would each have wasted a day:

  * The kill switch polarity was INVERTED from what the software assumed.
    PX4 read channel HIGH as KILLED, and the position the code called
    "released" was the killed one. Every rotor test looked silently dead.
  * PX4 had LATCHED flight termination, which clears only on a flight
    controller reboot -- not by toggling the switch, not by re-arming.

Neither is discoverable by reading configuration. Both are obvious in ten
seconds here.

Modes:

    --mode monitor    Live table of every RC channel plus PX4 state. Start here.
    --mode identify   Wiggle one switch; reports which channel index moved.
    --mode kill       Guided polarity measurement for the kill switch.
    --mode gates      Watch the two channels the RL runtime actually gates on.

    python3 scripts/hardware_rc_check.py --mode monitor

Preconditions for all modes: PROPELLERS OFF, vehicle restrained, transmitter
on, PX4 powered, and the uXRCE-DDS agent running (./interface.sh).
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import ActuatorArmed, InputRc, VehicleStatus

# PX4 1.16+ versions some topic names; vehicle_status is one of them.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "atmo"))
from atmo import px4_topics  # noqa: E402

# Must match atmo/rl_controller_hardware.py, which is what actually gates the
# runtime. Reading them from the same environment variables means this tool
# reports the deployed interpretation rather than a second opinion.
RC_MAX = int(os.getenv("ATMO_RL_RC_MAX", "1934"))
RC_MARGIN = int(os.getenv("ATMO_RL_RC_MARGIN", "100"))
OFFBOARD_CHANNEL = int(os.getenv("ATMO_RL_OFFBOARD_CHANNEL", "8"))
RL_CHANNEL = int(os.getenv("ATMO_RL_CHANNEL", "7"))
RC_PLAUSIBLE_MIN_US = int(os.getenv("ATMO_RL_RC_PLAUSIBLE_MIN_US", "900"))

GATE_THRESHOLD = RC_MAX - RC_MARGIN

ARM_REASON = {
    0: "transition_to_standby", 1: "rc_stick", 2: "rc_switch",
    3: "command_internal", 4: "command_external", 5: "mission_start",
    6: "safety_button", 7: "auto_disarm_land", 8: "auto_disarm_preflight",
    9: "KILL_SWITCH", 10: "LOCKDOWN", 11: "FAILURE_DETECTOR",
    12: "shutdown", 13: "unit_test",
}
NAV_STATE = {0: "manual", 1: "altctl", 2: "posctl", 10: "acro", 12: "descend",
             13: "TERMINATION", 14: "offboard", 15: "stab"}


class RcListener(Node):
    def __init__(self):
        super().__init__("atmo_rc_check")
        self.rc = None
        self.rc_count = 0
        self.status = None
        self.armed_msg = None
        self.create_subscription(
            InputRc, px4_topics.resolve(self, "input_rc"), self._rc,
            qos_profile_sensor_data)
        self.create_subscription(
            VehicleStatus, px4_topics.resolve(self, "vehicle_status"), self._status,
            qos_profile_sensor_data)
        # Optional: only present if the uXRCE dds_topics.yaml publishes it.
        # It carries the kill switch state directly, so prefer it when there.
        self.create_subscription(
            ActuatorArmed, "/fmu/out/actuator_armed", self._armed,
            qos_profile_sensor_data)

    def _rc(self, msg):
        self.rc = msg
        self.rc_count += 1

    def _status(self, msg):
        self.status = msg

    def _armed(self, msg):
        self.armed_msg = msg


def spin(node, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)


def wait_for_rc(node, seconds=8.0):
    print("Waiting for /fmu/out/input_rc ...")
    deadline = time.time() + seconds
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.rc is not None:
            print("RC stream up (%d channels).\n" % node.rc.channel_count)
            return True
    print("\nFAIL: no RC on /fmu/out/input_rc after %.0f s.\n" % seconds)
    print("Check, in this order:")
    print("  1. Is the uXRCE-DDS agent running?   ./interface.sh")
    print("  2. Is the transmitter on and the receiver bound?")
    print("  3. Does PX4 see RC at all?           listener input_rc")
    print("  4. Same ROS_DOMAIN_ID as the agent?  echo $ROS_DOMAIN_ID")
    return False


def gate_state(rc, channel):
    """Exactly what rl_controller_hardware._channel_high would decide."""
    if rc is None or channel < 0 or channel >= len(rc.values):
        return False, None
    value = int(rc.values[channel])
    if rc.rc_lost or rc.rc_failsafe:
        return False, value
    if value < RC_PLAUSIBLE_MIN_US:
        return False, value
    return value >= GATE_THRESHOLD, value


def px4_kill_state(node):
    """Return (killed, source). None when it cannot be determined.

    The kill field is NOT called the same thing across PX4 versions. Measured
    on this vehicle 2026-08-14 (PX4 1.17.0, git 8874d533): `listener
    actuator_armed` reports `lockdown` and `kill`, and has no `manual_lockdown`
    at all -- while the px4_msgs ActuatorArmed.msg vendored in this workspace
    still declares `manual_lockdown`. Try each in turn rather than assuming.
    """
    if node.armed_msg is not None:
        for field in ("kill", "manual_lockdown", "lockdown"):
            if hasattr(node.armed_msg, field):
                return bool(getattr(node.armed_msg, field)), "actuator_armed.%s" % field
    status = node.status
    if status is not None:
        if int(status.nav_state) == 13:
            return True, "nav_state=TERMINATION"
        if int(status.latest_arming_reason) == 9:
            return True, "latest_arming_reason=KILL_SWITCH"
        return False, "inferred from vehicle_status (no actuator_armed)"
    return None, "no PX4 state"


def print_px4(node):
    status = node.status
    if status is None:
        print("  PX4 state       (no /fmu/out/vehicle_status yet)")
        return
    armed = int(status.arming_state) == 2
    nav = NAV_STATE.get(int(status.nav_state), "state_%d" % status.nav_state)
    reason = ARM_REASON.get(
        int(status.latest_arming_reason), "reason_%d" % status.latest_arming_reason)
    killed, source = px4_kill_state(node)
    print("  armed           %s" % ("ARMED" if armed else "disarmed"))
    print("  nav_state       %s" % nav)
    print("  last arm reason %s" % reason)
    print("  failsafe        %s" % bool(status.failsafe))
    print("  preflight ok    %s" % bool(status.pre_flight_checks_pass))
    print("  KILLED          %s   [%s]" % (killed, source))
    if node.armed_msg is not None:
        a = node.armed_msg
        print("  lockdown        %s     force_failsafe %s" % (a.lockdown, a.force_failsafe))
    if int(status.nav_state) == 13:
        print("\n  *** PX4 IS IN FLIGHT TERMINATION. ***")
        print("  This LATCHES. Toggling the switch will not clear it and")
        print("  re-arming will not clear it. Reboot the flight controller.")


def mode_monitor(node, args):
    if not wait_for_rc(node):
        return 1
    print("Ctrl-C to stop. Move switches and watch which numbers change.\n")
    try:
        while rclpy.ok():
            spin(node, args.interval)
            rc = node.rc
            print("=" * 68)
            if rc is None:
                print("  RC LOST")
                continue
            values = [int(v) for v in rc.values[:rc.channel_count]]
            for start in range(0, len(values), 6):
                chunk = values[start:start + 6]
                labels = "".join("  ch%-2d " % (start + i + 1) for i in range(len(chunk)))
                numbers = "".join("%6d " % v for v in chunk)
                print("  " + labels)
                print("  " + numbers)
            print("  rc_lost %s   rc_failsafe %s   rssi %d   frames %d"
                  % (rc.rc_lost, rc.rc_failsafe, rc.rssi, node.rc_count))
            print_px4(node)
    except KeyboardInterrupt:
        pass
    return 0


def mode_identify(node, args):
    if not wait_for_rc(node):
        return 1
    print("Leave every switch alone. Sampling the resting position ...")
    spin(node, 2.0)
    base = [int(v) for v in node.rc.values[:node.rc.channel_count]]
    input("\nNow move ONE switch to its other extreme and HOLD it, then press Enter.")
    spin(node, 1.0)
    now = [int(v) for v in node.rc.values[:node.rc.channel_count]]
    moved = [(abs(now[i] - base[i]), i) for i in range(min(len(base), len(now)))]
    moved.sort(reverse=True)
    print("\nChannels by how much they changed:\n")
    for delta, index in moved[:5]:
        marker = ""
        if index == OFFBOARD_CHANNEL:
            marker = "   <- configured OFFBOARD gate"
        elif index == RL_CHANNEL:
            marker = "   <- configured RL gate"
        print("  index %-2d (ch%-2d)  %5d -> %5d   delta %5d%s"
              % (index, index + 1, base[index], now[index], delta, marker))
    if moved and moved[0][0] < 50:
        print("\n  Nothing moved much. Wrong switch, or the channel is unmapped")
        print("  in the transmitter's function menu.")
        return 1
    top = moved[0][1]
    print("\n  That switch is channel INDEX %d (ch%d on the transmitter)." % (top, top + 1))
    print("  The runtime gates on index %d (offboard) and index %d (RL)."
          % (OFFBOARD_CHANNEL, RL_CHANNEL))
    print("  Override with ATMO_RL_OFFBOARD_CHANNEL / ATMO_RL_CHANNEL.")
    return 0


def mode_gates(node, args):
    if not wait_for_rc(node):
        return 1
    print("Gate rule: a channel is HIGH at >= %d us (RC_MAX %d - margin %d),"
          % (GATE_THRESHOLD, RC_MAX, RC_MARGIN))
    print("and reads LOW on rc_lost, rc_failsafe, or below %d us.\n" % RC_PLAUSIBLE_MIN_US)
    single_gate = OFFBOARD_CHANNEL < 0
    if single_gate:
        print("SINGLE-GATE airframe (ATMO_RL_OFFBOARD_CHANNEL=%d): there is no"
              % OFFBOARD_CHANNEL)
        print("offboard switch. The RL switch is cycled twice instead, and it")
        print("alone must pass the deadman test.\n")
        print("Exercise the RL switch through both positions. Ctrl-C to stop.\n")
    else:
        print("Exercise both switches through every combination. Ctrl-C to stop.\n")
    try:
        while rclpy.ok():
            spin(node, args.interval)
            rl, rv = gate_state(node.rc, RL_CHANNEL)
            rl_text = ("%5d us" % rv) if rv is not None else "  --  "
            if single_gate:
                print("  RL[idx %d] %s  %s"
                      % (RL_CHANNEL, rl_text, "HIGH" if rl else "low "))
                continue
            offboard, ov = gate_state(node.rc, OFFBOARD_CHANNEL)
            print("  offboard[idx %d] %s  %s        RL[idx %d] %s  %s"
                  % (OFFBOARD_CHANNEL,
                     ("%5d us" % ov) if ov is not None else "  --  ",
                     "HIGH" if offboard else "low ",
                     RL_CHANNEL, rl_text,
                     "HIGH" if rl else "low "))
    except KeyboardInterrupt:
        pass
    if single_gate:
        print("\nThe RL gate must be reachable HIGH and low, and must read low")
        print("when you power the transmitter off. That last one is the deadman,")
        print("and on a single-gate airframe it is the only thing that stops the")
        print("tilt and drive RoboClaws.")
    else:
        print("\nBoth gates must be reachable HIGH and low, and must read low when")
        print("you power the transmitter off. That last one is the deadman.")
    return 0


def mode_kill(node, args):
    if not wait_for_rc(node):
        return 1
    print("""
KILL SWITCH POLARITY -- measured, not assumed.

PROPELLERS OFF. This mode commands nothing; it only reads what PX4 believes.
You are establishing which PHYSICAL switch position PX4 considers KILLED.
Do not assume the label on the transmitter agrees.
""")
    readings = []
    for label in ("position A (however it sits now)", "the OTHER position"):
        input("Put the kill switch in %s, then press Enter." % label)
        spin(node, 1.5)
        rc = node.rc
        killed, source = px4_kill_state(node)
        channel_values = [int(v) for v in rc.values[:rc.channel_count]] if rc else []
        readings.append((label, killed, source, channel_values))
        print("  PX4 reports killed = %s   [%s]\n" % (killed, source))

    print("=" * 68)
    (label_a, killed_a, source, values_a) = readings[0]
    (label_b, killed_b, _, values_b) = readings[1]
    print("Determined via: %s\n" % source)
    if killed_a is None or killed_b is None:
        print("FAIL: could not read PX4's kill state at all.")
        print("Neither /fmu/out/actuator_armed nor vehicle_status was usable.")
        print("Add actuator_armed to the uXRCE dds_topics.yaml, or read it in")
        print("the mavlink shell:  listener actuator_armed")
        return 1
    if killed_a == killed_b:
        print("FAIL: PX4 reported killed=%s in BOTH positions." % killed_a)
        print("Either the switch is not mapped to RC_MAP_KILL_SW, or PX4 has")
        print("LATCHED flight termination. Check the nav_state line below; if it")
        print("says TERMINATION, reboot the flight controller and rerun.")
        print_px4(node)
        return 1

    killed_label = label_a if killed_a else label_b
    changed = [
        (abs(values_a[i] - values_b[i]), i)
        for i in range(min(len(values_a), len(values_b)))
    ]
    changed.sort(reverse=True)
    print("PASS: the kill switch works in both directions.")
    print("  KILLED   in: %s" % killed_label)
    print("  released in: %s" % (label_b if killed_a else label_a))
    if changed and changed[0][0] > 50:
        index = changed[0][1]
        print("  kill channel index %d (ch%d): %d us <-> %d us"
              % (index, index + 1, values_a[index], values_b[index]))
        high_is_killed = (values_a[index] > values_b[index]) == bool(killed_a)
        print("  PX4 treats %s as KILLED." % ("HIGH" if high_is_killed else "LOW"))
    print("\nWrite this down. It is the number the m4 vehicle had backwards.")
    print("Leave the switch in the KILLED position until a test asks otherwise.")
    return 0


MODES = {"monitor": mode_monitor, "identify": mode_identify,
         "gates": mode_gates, "kill": mode_kill}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=sorted(MODES), default="monitor")
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = RcListener()
    try:
        return MODES[args.mode](node, args)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
