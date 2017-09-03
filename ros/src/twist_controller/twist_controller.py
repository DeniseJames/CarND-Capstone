
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import math


class Controller(object):
    def __init__(self, speed_controller, yaw_controller):
        # TODO: Implement
        self.speed_controller = speed_controller
        self.yaw_controller = yaw_controller
        

    def control(self, linear_velocity, angular_velocity, current_velocity, enable_dbw):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not enable_dbw:
            # Reset PID
            return 0, 0, 0

        # Calculate throttle, and under which the linear velocity after 2 seconds
        throttle, brake, final_velocity = self.speed_controller.get_control(linear_velocity,
                                                                            current_velocity,
                                                                            delta_t=2.)

        steer = self.yaw_controller.get_steering(final_velocity, angular_velocity, current_velocity)
        steer *= 180. / math.pi
        return throttle, brake, steer
