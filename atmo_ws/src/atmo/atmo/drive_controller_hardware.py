import os
from time import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import InputRc
from std_msgs.msg import Bool

# roboclaw and jetson
from atmo.mpc.DriveControllerBase import DriveControllerBase
from atmo.mpc.roboclaw_3 import Roboclaw
from atmo.mpc.parameters import params_
from atmo.roboclaw_safety import gate_open, stop_all

RC_TIMEOUT_S = float(os.getenv("ATMO_RL_RC_TIMEOUT_S", "0.5"))
# Open-loop straightness trim for the left wheel; see move_left_wheel().
# Default is 1.0 since 2026-08-17: the left side's extra resistance was traced
# to a mechanical cause and fixed, so the 1.35x trim it motivated would now
# OVER-drive the left wheel and curve the vehicle the other way. The knob
# stays for future trims; re-measure with a straight-line run before using it.
LEFT_DUTY_SCALE = float(os.getenv("ATMO_DRIVE_LEFT_SCALE", "1.0"))

# get parameters
min                    = params_.get('min')
max                    = params_.get('max')
dead                   = params_.get('dead')
Ts                     = params_.get('Ts_drive_controller')
drive_roboclaw_address = params_.get('drive_roboclaw_address')
# pitch_channel / roll_channel / offboard_channel are deliberately NOT read
# here any more. They were the stick-to-wheel path; see rc_listener_callback.
# The gate now comes from atmo.roboclaw_safety, which uses the same channel
# indices and conventions as rl_controller_hardware.

class DriveControllerHardware(DriveControllerBase):
    def __init__(self): 
        super().__init__()

        self.subscription = self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            self.rc_listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.manual_override_subscription = self.create_subscription(
            Bool,
            '/atmo/rl/manual_override',
            self.manual_override_callback,
            qos_profile_sensor_data)

        # initialize drive speed and turn speed

        # Manual (stick-driven) control is disabled; see rc_listener_callback.
        self.manual = True
        self.manual_override = False
        # Fail closed until RC arrives and proves the gate is raised.
        self.gate_open = False
        self.rc_stamp = 0.0

        # roboclaw stuff
        self.address = 0x80
        self.rc = Roboclaw(drive_roboclaw_address,115200)
        self.rc.Open()

    def rc_listener_callback(self, msg):
        # MANUAL (stick-driven) CONTROL IS DISABLED ON THIS VEHICLE.
        #
        # This used to read the pitch and roll sticks straight into wheel
        # speed. Measured 2026-08-14: `offboard_channel` indexed ch9, a channel
        # with no switch on it, so `offboard_automatic` was permanently False
        # and this node was permanently in the stick branch. The pitch stick
        # rests at 1527 against a deadband centre of 1514 -- a 13 us trim
        # offset -- which normalize()/map_speed() turned into a standing
        # command of speed 3 to both wheels from the moment the node started.
        # That is what drove the wheels with nothing commanded, and no gate
        # stood in front of it. The branch now commands ZERO; see update().
        self.manual = True

        self.rc_stamp = time()
        self.gate_open = (
            not self.manual_override
            and gate_open(msg.values, msg.rc_lost, msg.rc_failsafe)
        )

    def manual_override_callback(self, msg):
        # Override now means HOLD ZERO, not "hand back to the stick".
        self.manual_override = bool(msg.data)
        if self.manual_override:
            self.gate_open = False

    # normalize() is gone with the stick path. It mapped a raw RC pulse width
    # to a wheel speed, and it had no deadband -- the pitch stick's 13 us trim
    # offset came out as a standing speed-3 command to both wheels. Nothing
    # should be turning an RC value into a wheel speed in this node.

    def map_speed(self,speed_normalized):
        return int(127*speed_normalized)

    def stop(self):
        """Zero every motor. The ONLY software stop this board has."""
        return stop_all(self.rc, self.address, self.get_logger())

    def on_shutdown(self):
        # Order matters and used not to exist. This previously closed the port
        # and sent nothing, so the board stayed latched at its last commanded
        # speed with no host attached -- which is exactly what happened on
        # 2026-08-14. Stop FIRST, then close.
        if self.stop():
            self.get_logger().info("drive motors zeroed")
        else:
            self.get_logger().error(
                "DRIVE MOTORS MAY STILL BE RUNNING: no stop command was "
                "acknowledged. Use scripts/estop_roboclaw.py, then the battery."
            )
        try:
            self.rc._port.close()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("port close failed: %s" % exc)
        self.get_logger().info("port closed !")

    def move_right_wheel(self, speed):
        if speed > 0:
            if speed >= 127:
                speed = 126
            self.rc.BackwardM1(self.address, abs(speed))
        else:
            if speed <= -127:
                speed = -126
            self.rc.ForwardM1(self.address, abs(speed))

    def move_left_wheel(self, speed):
        # LEFT_DUTY_SCALE is a straightness trim for left/right asymmetry.
        # There are NO encoders on this board, so nothing closes a loop --
        # any trim here is open loop, duty-ratio at one duty on one surface.
        # The historical 1.35x (2026-08-14, trimmed by feel) compensated a
        # mechanical drag on the left side that has since been found and
        # fixed, so the default is back to 1.0. If the vehicle curves on a
        # measured straight-line run, trim via ATMO_DRIVE_LEFT_SCALE.
        speed = speed * LEFT_DUTY_SCALE
        if speed > 0:
            if speed >= 127:
                speed = 126
            self.rc.ForwardM2(self.address, int(abs(speed)))
        else:
            if speed <= -127:
                speed = -126
            self.rc.BackwardM2(self.address, int(abs(speed)))

    def update(self):
        # Fail closed on a stale RC stream. PX4's kill does not reach these
        # wheels; this and the gate are the only things that stop them.
        if time() - self.rc_stamp > RC_TIMEOUT_S:
            self.stop()
            return
        if not self.gate_open:
            # Covers manual mode (now zero), a lowered gate, and kill engaged.
            self.stop()
            return
        lin_vel = -self.map_speed(self.drive_speed)
        ang_vel = -self.map_speed(self.turn_speed)
        self.move_right_wheel(lin_vel + ang_vel)
        self.move_left_wheel(lin_vel - ang_vel)
     
def main(args=None):
    rclpy.init(args=args)
    drive_controller = DriveControllerHardware()
    drive_controller.get_logger().info("Starting DriveControllerHardware node...")
    # rclpy.spin() raises KeyboardInterrupt on Ctrl-C. Without this try/finally
    # every line after it was skipped, so on_shutdown() never ran and the board
    # was left latched. The finally is the point: the motors must be zeroed on
    # EVERY exit path, not just the clean one.
    try:
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drive_controller.on_shutdown()
        drive_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
