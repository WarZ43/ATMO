"""Stopping a RoboClaw, and why it needs its own module.

A RoboClaw latches its last command **indefinitely**. There is no command
timeout configured on this vehicle's boards, so once told to move they move
until told otherwise or until power is removed. Closing the serial port does
not stop them. Killing the process does not stop them. Dropping the RC gate
does not stop them, because the gate only prevents a *live* node from sending
commands -- a dead node with a latched board ignores it completely.

Measured 2026-08-14: both RoboClaws kept driving after every node had exited
and both ports were free. The only things that stopped them were a zero
written over the serial port, and disconnecting the battery.

That makes the zero-on-exit path load-bearing rather than cleanup. It is the
only software stop that exists. Everything here exists to make sure it runs.

`scripts/estop_roboclaw.py` is the standalone version, for when no node is
alive to run this at all.
"""

import os

# RoboClaw packet-serial address. Both boards on this vehicle use the default.
DEFAULT_ADDRESS = 0x80

# 0-BASED indices into InputRc.values. Same convention and same measured values
# as rl_controller_hardware; see the README.
RL_CHANNEL = int(os.getenv("ATMO_RL_CHANNEL", "7"))
KILL_CHANNEL = int(os.getenv("ATMO_KILL_CHANNEL", "12"))
RC_MAX = int(os.getenv("ATMO_RL_RC_MAX", "1934"))
RC_MARGIN = int(os.getenv("ATMO_RL_RC_MARGIN", "100"))
RC_PLAUSIBLE_MIN_US = int(os.getenv("ATMO_RL_RC_PLAUSIBLE_MIN_US", "900"))
# Midpoint of the kill channel. Measured 2026-08-14: ch13 (index 12) reads
# 944 us with the lever physically UP (released) and 2084 us with it DOWN
# (KILLED). Anything at or above this threshold reads as killed.
KILL_LOW_THRESHOLD = int(os.getenv("ATMO_KILL_LOW_US", "1500"))
# Which end of the kill channel means KILLED, in MICROSECONDS, not lever
# position -- the two are reversed on this airframe. See kill_engaged().
KILL_ACTIVE_HIGH = os.getenv("ATMO_KILL_ACTIVE_HIGH", "1").lower() in {
    "1", "true", "yes", "on"}


def _value(values, index):
    if index < 0 or index >= len(values):
        return None
    return int(values[index])


def kill_engaged(values, rc_lost=False, rc_failsafe=False):
    """True when the kill switch is in the killed position, or RC is gone.

    PX4's kill cuts the ROTORS. It does not reach the RoboClaws at all -- they
    are commanded only from the companion. This is what carries the kill across
    to tilt and drive, so it must read killed whenever it cannot prove
    otherwise: an unknown channel, an implausible pulse width, or lost RC.
    Those three fail closed regardless of polarity.

    POLARITY, settled 2026-08-14. `ATMO_KILL_ACTIVE_HIGH` selects which end
    is killed and defaults to HIGH, which is both PX4's usual convention and
    what this airframe does:

        lever physically UP    -> ch13 =  944 us  -> NOT killed
        lever physically DOWN  -> ch13 = 2084 us  -> KILLED

    THE CHANNEL IS REVERSED RELATIVE TO THE LEVER. That is the whole trap,
    and it produced a real inversion in this file: the operator's "kill high
    is no kill, kill low is kill" describes the LEVER, and it was read as
    describing microseconds. The resulting active-low logic called a released
    switch engaged, which blocked preflight, and would have called an engaged
    switch released, which is the direction that matters.

    When re-checking, state which axis you mean. `listener actuator_armed 1`
    in the mavlink shell reports `kill:` against a PHYSICAL position; the
    microsecond value is a separate fact that may not agree in sense.
    """
    if rc_lost or rc_failsafe:
        return True
    value = _value(values, KILL_CHANNEL)
    if value is None or value < RC_PLAUSIBLE_MIN_US:
        return True
    if KILL_ACTIVE_HIGH:
        return value >= KILL_LOW_THRESHOLD
    return value < KILL_LOW_THRESHOLD


def gate_open(values, rc_lost=False, rc_failsafe=False):
    """True only when the RL gate is HIGH and the kill is not engaged.

    Fails closed on every unknown, exactly like the gates in
    rl_controller_hardware. The RoboClaws have no other stop.
    """
    if kill_engaged(values, rc_lost, rc_failsafe):
        return False
    value = _value(values, RL_CHANNEL)
    if value is None or value < RC_PLAUSIBLE_MIN_US:
        return False
    return value >= RC_MAX - RC_MARGIN


def stop_all(rc, address=DEFAULT_ADDRESS, logger=None):
    """Command every motor on `rc` to zero, by every route the board accepts.

    Deliberately redundant. Which of these a board honours depends on the mode
    it was last commanded in -- a board driven with ForwardM1 does not
    necessarily stop on DutyM1 -- and this runs in the path where getting it
    wrong means a motor nobody can stop without pulling the battery. Each call
    is independently guarded so that one unsupported opcode cannot prevent the
    others from being sent.

    Returns True if at least one stop command was acknowledged.
    """
    sent = False
    # Forward/Backward at 0 is what the drive and tilt nodes themselves use,
    # so it is guaranteed to be supported by whatever mode they left the board
    # in. Send these first and unconditionally.
    for name in ("ForwardM1", "BackwardM1", "ForwardM2", "BackwardM2"):
        sent = _try(rc, name, (address, 0), logger) or sent
    # Speed and duty cover boards left in a closed-loop or PWM mode.
    for name in ("SpeedM1", "SpeedM2", "DutyM1", "DutyM2"):
        sent = _try(rc, name, (address, 0), logger) or sent
    return sent


def _try(rc, name, args, logger):
    method = getattr(rc, name, None)
    if method is None:
        return False
    try:
        method(*args)
        return True
    except Exception as exc:  # noqa: BLE001 - never let one opcode block the rest
        if logger is not None:
            logger.warn("RoboClaw %s failed during stop: %s" % (name, exc))
        return False
