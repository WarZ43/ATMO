# ROS imports
import rclpy

# Standard imports
from os import getenv

# Numpy imports
from numpy import clip, pi, sign

# Morphing Lander imports
from atmo.mpc.TiltControllerBase import TiltControllerBase
from atmo.mpc.parameters import params_

v_max_absolute = params_['v_max_absolute']
tilt_lower = 0.0
tilt_upper = min(
    float(getenv("ATMO_RL_SIM_TILT_ACTUATOR_UPPER", str(85.0 * pi / 180.0))),
    pi / 2.0,
)

class TiltSim(TiltControllerBase): 
    def __init__(self):
        super().__init__()
        self.tilt_angle = float(clip(self.tilt_angle, tilt_lower, tilt_upper))
 
    def reset_encoder_trigger(self):
        pass

    def get_current_tilt_angle(self):
        self.tilt_angle = float(clip(self.tilt_angle, tilt_lower, tilt_upper))
        return self.tilt_angle

    def update(self):
        tilt_vel = float(sign(clip(self.tilt_vel, -1.0, 1.0)))
        if (self.tilt_angle <= tilt_lower and tilt_vel < 0.0) or (
            self.tilt_angle >= tilt_upper and tilt_vel > 0.0
        ):
            tilt_vel = 0.0
        self.tilt_vel = tilt_vel
        self.tilt_angle = float(
            clip(self.tilt_angle + float(self.Ts) * tilt_vel * v_max_absolute, tilt_lower, tilt_upper)
        )

def main(args=None):
    rclpy.init(args=args)
    print("Spinning TiltSim node \n")
    tilt_sim = TiltSim()
    rclpy.spin(tilt_sim)
    tilt_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
