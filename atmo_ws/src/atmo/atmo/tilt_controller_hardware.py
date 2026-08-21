import os
from time import time

# Numpy imports
from numpy import interp, array, deg2rad, rad2deg

# ROS imports
import rclpy
from rclpy.qos import qos_profile_sensor_data

# Message imports
from px4_msgs.msg import InputRc
from std_msgs.msg import Bool

# Morphing Lander imports
from atmo.mpc.TiltControllerBase import TiltControllerBase
from atmo.mpc.roboclaw_3 import Roboclaw
from atmo.mpc.parameters import params_
from atmo.roboclaw_safety import gate_open, kill_engaged, stop_all

# The RoboClaws are outside PX4's kill entirely, so a stale RC stream must stop
# them here. Same 0.5 s as rl_controller_hardware.
RC_TIMEOUT_S = float(os.getenv("ATMO_RL_RC_TIMEOUT_S", "0.5"))

min                    = params_['min']
max                    = params_['max']
dead                   = params_['dead']
Ts                     = params_.get('Ts_tilt_controller')
tilt_roboclaw_address  = params_.get('tilt_roboclaw_address')
# tilt_channel is deliberately no longer read: it was the LS trim lever that
# drove this motor directly, ungated. See rc_listener_callback.
encoder_channel        = params_.get('encoder_channel')
offboard_channel       = params_.get('offboard_channel')

_TILT_DEBUG = os.getenv("ATMO_TILT_DEBUG", "0").lower() in ("1", "true", "yes", "on")
# Mirrors rl_controller_hardware.ACTION_AUTOSTART for the bench: this node's
# gate becomes RC liveness (kill released) instead of the RL switch. Never set
# for flight sessions.
_ACTION_AUTOSTART = os.getenv("ATMO_RL_ACTION_AUTOSTART", "0").lower() in (
    "1", "true", "yes", "on")


class TiltHardware(TiltControllerBase):
    def __init__(self):
        super().__init__()

        # Subscriptions
        self.rc_subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.rc_subscription        # rc subscription
        self.manual_override_subscription = self.create_subscription(
            Bool,
            '/atmo/rl/manual_override',
            self.manual_override_callback,
            qos_profile_sensor_data)

        # Configure RC inputs
        self.min = min 
        self.max = max 
        self.dead = dead 

        # Manual (stick-driven) control is disabled; see rc_listener_callback.
        self.manual = True
        self.manual_override = False
        # Fail closed until RC actually arrives and proves the gate is raised.
        self.gate_open = False
        self.rc_stamp = 0.0

        # Initialize roboclaw at given address
        self.address = 0x80
        self.rc = Roboclaw(tilt_roboclaw_address,115200)
        self.rc.Open()

        # Set encoder count to zero AT WHEREVER THE ARM CURRENTLY IS. What
        # zero MEANS is decided by ATMO_TILT_HOME below: the default since
        # 2026-08-17 assumes the worst case (top / drive, 90 deg) because a
        # power cycle can happen with the arm parked on a kill switch and the
        # node has no way to know. Under "fly" it means the fly hard stop,
        # which is only true after a verified homing.
        #
        # ATMO_TILT_PRESERVE_ENC=1 skips this zero: the operator has homed
        # (scripts/home_tilt.py, SetEncM2(0) at the fly stop) and the count
        # currently on the board is that frame. Without it, launching the
        # stack silently discarded a verified homing -- measured 2026-08-18:
        # arm homed at fly, driven to 85 deg, node boot re-zeroed there.
        # Only combine with ATMO_TILT_HOME=fly.
        if os.getenv("ATMO_TILT_PRESERVE_ENC", "0") != "1":
            self.rc.SetEncM2(self.address,0)
        self.reset_encoder = 0

        # Set pin functions for motor 2 (M2) to go to zero when it reaches home (limit switch)
        # This is why the raw count jumps without anything commanding the
        # motor: reaching home re-zeroes the encoder in hardware.
        self.rc.SetPinFunctions(self.address,0x00,0x62,0x62)

        # Encoder/angle calibration.
        #
        # The raw table below was collected from the OPPOSITE mechanical end,
        # descending 90 deg -> 0.8 deg as counts ascend. Using it directly --
        # which this file did until 2026-08-14 -- puts 0 counts at 90 deg and
        # makes the angle run backwards. Reflect both axes so that 0 counts is
        # 0 deg at the fly configuration and the angle ascends with count,
        # preserving the nonlinear shape of the original measurement.
        calibration_encoder_data = array([0,3696,5551,7647,8886,10118,11062,11982,12846,13957,14885,15629,16549,17037,17749,19029,19645,20989,21453,22357,22989,23517,24013,24605,25437,25973,26325])
        # The January table's 26325-count span NO LONGER MATCHES the mechanism:
        # measured 2026-08-17 (fly stop to the drive-end kill switch, by
        # motion) the full span is ~22200 counts. Running on the stale span
        # under-reports the angle by ~18%, which is exactly how the 85 deg
        # software bound watched the arm hit the physical 90 deg switch while
        # reading 57. Scale the table to the measured span; the nonlinear
        # SHAPE is kept, only the span is corrected. Re-measure after any
        # drivetrain work: ATMO_TILT_SPAN_COUNTS overrides.
        measured_span = float(os.getenv("ATMO_TILT_SPAN_COUNTS", "22200"))
        calibration_encoder_data = calibration_encoder_data * (
            measured_span / float(calibration_encoder_data[-1]))
        raw_angle_data_deg = array([90,86.6,84.0,78.6,74.2,70.5,66.5,63.5,59.7,55.3,51.3,48.0,44.2,41.8,38.4,32.8,30.0,24.2,22.0,18.0,14.8,12.9,10.8,8.2,4.7,2.8,0.8])
        # WHICH END THE ENCODER WAS ZEROED AT decides the calibration.
        #
        #   fly             -- zero at the FLY hard stop. Counts go NEGATIVE
        #                      moving toward drive, so negate before lookup and
        #                      use the reflected table (0 counts -> 0 deg).
        #   drive (default) -- zero at the DRIVE hard stop (worst-case boot
        #                      assumption; see below). Counts go POSITIVE
        #                      moving toward fly, so use the raw count and the
        #                      ORIGINAL table unchanged: it already reads
        #                      0 counts -> 90 deg descending to 0.8 deg, which
        #                      is exactly drive -> fly.
        #
        # The angle convention itself never changes and matches training
        # (ATMO_SPEC: hover/fly = 0 rad, landing/drive = pi/2).
        # DEFAULT IS "drive", DELIBERATELY, since 2026-08-17: the tilt travel
        # ends carry hardware kill switches that cut the motor circuit, and a
        # power cycle wipes the encoder, so the node cannot know where the arm
        # is. Assume the WORST CASE: the last motion was upward and the arm
        # sits at the TOP (drive, ~90 deg) kill switch. With zero mapped to
        # 90 deg, the max_tilt_angle guard below blocks every upward command
        # from the first tick, and upward motion only unblocks once the arm
        # has measurably descended below the bound. Booting a vehicle that is
        # actually at fly under this assumption merely under-reports nothing
        # dangerous: upward stays blocked until real downward motion is seen.
        # Set ATMO_TILT_HOME=fly only after a verified homing at the fly stop.
        self.tilt_home = os.getenv("ATMO_TILT_HOME", "drive").strip().lower()
        if self.tilt_home not in ("fly", "drive"):
            raise ValueError("ATMO_TILT_HOME must be 'fly' or 'drive', got %r" % self.tilt_home)
        if self.tilt_home == "drive":
            self.encoder_data = calibration_encoder_data
            self.angle_data = deg2rad(raw_angle_data_deg)
            self.encoder_sign = 1.0
        else:
            self.encoder_data = calibration_encoder_data[-1] - calibration_encoder_data[::-1]
            self.angle_data = deg2rad(raw_angle_data_deg[::-1] - raw_angle_data_deg[-1])
            self.encoder_sign = -1.0

        self._last_tilt_print = 0.0

        # HARD software bound for the high-tilt end: never command past this,
        # regardless of what the hardware kill switches do. 85 deg keeps a
        # margin below the ~90 deg top switch so the software stops the arm
        # before the hardware ever has to. Together with the assume-top boot
        # default above (zero counts = 90 deg), a fresh power-up starts ABOVE
        # this bound and upward motion is blocked until the arm has actually
        # descended below it.
        self.max_tilt_angle = deg2rad(float(os.getenv("ATMO_TILT_MAX_DEG", "85.0")))

        # limit tilt
        self.limit_tilt = deg2rad(1.0)

        # Command slew limit; see update(). State starts at zero so the first
        # command after enable ramps from rest instead of stepping.
        self.slew_per_s = float(os.getenv("ATMO_TILT_SLEW_PER_S", "4.0"))
        self.commanded_vel = 0.0

        # --- closed-loop velocity mode -------------------------------------
        # Duty commands (ForwardM2/BackwardM2) are open loop: the same duty
        # gives wildly different speeds depending on where the arm is, because
        # gravity loads it differently. Measured 2026-08-14: breakaway is duty
        # ~45 running away from fly and ~100 running back toward it, and duty
        # 100 into the hard stop wedges the arm. Velocity targets fix that --
        # the board holds counts/s regardless of load.
        #
        # This REQUIRES the board's velocity PID and QPPS to be set. They ship
        # at zero here (measured: P=I=D=0, QPPS=0), and a SpeedM2 command with
        # QPPS=0 is ACCEPTED and does NOTHING -- exactly the silent failure
        # this file has been full of. So configure, then READ BACK, and refuse
        # to use velocity mode if the readback does not stick.
        self.velocity_mode = os.getenv("ATMO_TILT_VELOCITY_MODE", "0").lower() in (
            "1", "true", "yes", "on")
        # Measured top speed: ~2780 counts/s at duty 126.
        self.tilt_qpps = int(os.getenv("ATMO_TILT_QPPS", "2800"))
        if self.velocity_mode:
            self._configure_velocity_pid()

    def _configure_velocity_pid(self):
        p = float(os.getenv("ATMO_TILT_VEL_P", "1.0"))
        i = float(os.getenv("ATMO_TILT_VEL_I", "0.5"))
        d = float(os.getenv("ATMO_TILT_VEL_D", "0.25"))
        try:
            self.rc.SetM2VelocityPID(self.address, p, i, d, self.tilt_qpps)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("SetM2VelocityPID raised: %s" % exc)
            self.velocity_mode = False
            return
        readback = None
        try:
            readback = self.rc.ReadM2VelocityPID(self.address)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("ReadM2VelocityPID raised: %s" % exc)
        # readback is [ok, P, I, D, QPPS]
        if not readback or not readback[0] or int(readback[4]) == 0:
            self.get_logger().error(
                "Velocity PID did not stick (readback=%s). Falling back to DUTY "
                "mode -- a SpeedM2 command with QPPS=0 is accepted and does "
                "nothing, which would look like a dead motor." % (readback,))
            self.velocity_mode = False
            return
        self.tilt_qpps = int(readback[4])
        self.get_logger().warn(
            "Tilt velocity mode ON: P=%.3f I=%.3f D=%.3f QPPS=%d"
            % (readback[1], readback[2], readback[3], self.tilt_qpps))

    def rc_listener_callback(self, msg):
        # reset encoder
        self.reset_encoder = msg.values[encoder_channel]

        # MANUAL (stick-driven) CONTROL IS DISABLED ON THIS VEHICLE. It used to
        # read a trim lever off `tilt_channel` and drive the motor from it,
        # which is a stick-to-motor path that no gate stood in front of. The
        # branch still exists so the structure is recognisable, but it commands
        # ZERO. See update().
        self.manual = True

        # Commanding is allowed only while the RL gate is HIGH and the kill is
        # disengaged. This replaces `offboard_channel`, which indexed ch9 -- a
        # channel with no switch on it -- so `offboard_automatic` could never
        # become True and this node ran permanently in the stick-driven branch.
        self.rc_stamp = time()
        if _ACTION_AUTOSTART:
            # Bench autostart (see rl_controller_hardware.ACTION_AUTOSTART):
            # the gate is RC LIVENESS -- kill released, RC present -- with no
            # RL-switch position required. Every fail-closed path in
            # kill_engaged (lost RC, failsafe, implausible pulse) still
            # closes the gate instantly, as does the RC-stamp watchdog.
            self.gate_open = (
                not self.manual_override
                and not kill_engaged(msg.values, msg.rc_lost, msg.rc_failsafe)
            )
            return
        self.gate_open = (
            not self.manual_override
            and gate_open(msg.values, msg.rc_lost, msg.rc_failsafe)
        )

    def manual_override_callback(self, msg):
        # Override now means HOLD ZERO, not "hand back to the stick".
        self.manual_override = bool(msg.data)
        if self.manual_override:
            self.gate_open = False

    # normalize() is gone with the stick path: it turned the LS trim lever into
    # a tilt-motor speed, ungated. Nothing should map an RC value to motor
    # speed in this node.

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)

    def stop(self):
        """Zero every motor. The ONLY software stop this board has.

        This used to send ForwardM2(0) alone, which leaves M1 latched and
        assumes the board is in the forward-command mode. See
        atmo/roboclaw_safety.py for why that is not good enough.

        Stops are IMMEDIATE, never slew-limited; the slew state resets so
        the next command after a stop ramps from rest.
        """
        self.commanded_vel = 0.0
        return stop_all(self.rc, self.address, self.get_logger())

    def spin_motor(self, tilt_speed):
        """tilt_speed in [-1, 1]. POSITIVE INCREASES THE TILT ANGLE.

        The sign convention here was inverted before 2026-08-14: this file
        mapped tilt_speed < 0 to ForwardM2 and called it "go up". Measured on
        the vehicle, ForwardM2 drives the arm AWAY from the fly configuration,
        i.e. it INCREASES the angle, so positive commands belong on Forward.
        The old mapping inverted every tilt command the policy issued.
        """
        # Low end: at the fly configuration, refuse to drive further down.
        # The hardware limit switch also guards this end, in hardware.
        if self.tilt_angle < self.limit_tilt and tilt_speed < 0:
            tilt_speed = 0.0
        # High end: nothing in hardware guards this, so it must be done here.
        if tilt_speed > 0 and self.tilt_angle >= self.max_tilt_angle:
            self.get_logger().warn(
                "upper tilt software limit reached (%.1f deg >= %.1f deg); "
                "blocking positive tilt command"
                % (rad2deg(self.tilt_angle), rad2deg(self.max_tilt_angle)),
                throttle_duration_sec=0.5,
            )
            tilt_speed = 0.0

        if self.velocity_mode:
            # Closed loop. Sign follows the same convention as duty mode:
            # positive tilt_speed increases the angle, which is the direction
            # ForwardM2 drives, which is NEGATIVE encoder counts -- so the
            # counts/s target is negated.
            target_cps = int(-tilt_speed * self.tilt_qpps)
            if target_cps == 0:
                self.stop()
            else:
                self.rc.SpeedM2(self.address, target_cps)
            return

        motor_speed = self.map_speed(abs(tilt_speed))
        if motor_speed == 0:
            self.stop()
            return
        if motor_speed >= 127:
            motor_speed = 126  # weird bug not sure why this is needed
        if tilt_speed > 0:     # increase tilt angle, away from fly config
            self.rc.ForwardM2(self.address, motor_speed)
        else:
            self.rc.BackwardM2(self.address, motor_speed)

    def on_shutdown(self):
        # Stop FIRST, then close. Closing the port sends nothing and leaves the
        # board latched at its last command.
        if self.stop():
            self.get_logger().info("tilt motor zeroed")
        else:
            self.get_logger().error(
                "TILT MOTOR MAY STILL BE RUNNING: no stop command was "
                "acknowledged. Use scripts/estop_roboclaw.py, then the battery."
            )
        try:
            self.rc._port.close()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("port close failed: %s" % exc)
        self.get_logger().info("port closed !")

    def reset_encoder_trigger(self):
        if (self.reset_encoder == self.max):
            self.rc.SetEncM2(self.address,0)

    def get_current_tilt_angle(self):
        # compute and print current tilt angle
        enc_count = self.rc.ReadEncM2(self.address)
        # NEGATE. The encoder counts negative as the arm tilts away from the
        # fly configuration -- measured 2026-08-14, ForwardM2 drove 0 -> -9699
        # while the arm swung roughly 43 deg. Feeding the raw count to interp
        # clamps below the table's first entry and returns a CONSTANT angle,
        # which is what this file did: it reported 90 deg no matter where the
        # arm actually was.
        raw_encoder_count = enc_count[1]
        tilt_encoder_count = self.encoder_sign * raw_encoder_count
        self.tilt_angle = float(interp(tilt_encoder_count, self.encoder_data, self.angle_data))
        # This runs at Ts_tilt_controller (~143 Hz). Printing every cycle buries
        # anything else on the console and costs real time on the control loop,
        # so it is throttled to once a second. Set ATMO_TILT_DEBUG=1 for the
        # firehose, including the raw encoder count, when calibrating the table.
        if _TILT_DEBUG:
            print(f"tilt angle: {rad2deg(self.tilt_angle):.2f} deg  raw enc {raw_encoder_count}  lookup {tilt_encoder_count}")
        else:
            now = time()
            if now - self._last_tilt_print >= 1.0:
                self._last_tilt_print = now
                print(f"tilt angle is: {rad2deg(self.tilt_angle):.2f} deg")
        return self.tilt_angle

    def update(self):
        # Fail closed on a stale RC stream. Without this the gate keeps
        # whatever value arrived last, so powering the transmitter off leaves
        # this node commanding. PX4's kill does not cover this motor.
        if time() - self.rc_stamp > RC_TIMEOUT_S:
            self.stop()
            return
        if not self.gate_open:
            # Covers manual mode (now zero), a lowered gate, and kill engaged.
            self.stop()
            return
        # SLEW LIMIT the commanded speed. A step reversal is the worst case
        # this mechanism and its supply see (plugging current plus the full
        # regen surge -- the mechanism that killed the drive board), and the
        # worm gear takes the same hit mechanically. Limit how fast the
        # command may CHANGE; the value itself is untouched once reached.
        # ATMO_TILT_SLEW_PER_S is full-scale units per second: 4.0 means a
        # full -1 -> +1 reversal takes 0.5 s.
        max_step = self.slew_per_s * self.Ts
        delta = self.tilt_vel - self.commanded_vel
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self.commanded_vel += delta
        self.spin_motor(self.commanded_vel)

def main(args=None):
    rclpy.init(args=args)
    tilt_controller = TiltHardware()
    tilt_controller.get_logger().info("Starting TiltHardware node...")
    # rclpy.spin() raises KeyboardInterrupt on Ctrl-C. Without this try/finally
    # every line after it was skipped, so on_shutdown() never ran and the board
    # was left latched at its last command with no host attached.
    try:
        rclpy.spin(tilt_controller)
    except KeyboardInterrupt:
        pass
    finally:
        tilt_controller.on_shutdown()
        tilt_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
