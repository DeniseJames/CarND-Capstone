
MAX_ACCEL_TORQUE = 2000.
MAX_BRAKE_TORQUE = 20000.

import math


class SpeedController(object):
    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

    def get_control(self, target_speed, current_speed, delta_t=1.):
        """
        Computes linear controls

        target_speed: target speed, not necessarily to be achievable
        current_speed: current speed
        delta_t: lookahead time for prediction
        """
        global MAX_ACCEL_TORQUE, MAX_BRAKE_TORQUE
        delta_v = target_speed - current_speed
        accel = self.get_acceleration(delta_v, delta_t)
        final_speed = current_speed + accel * delta_t
        torque = self.vehicle_mass * accel * self.wheel_radius
        if torque > 0:
            return min(1., torque / MAX_ACCEL_TORQUE), 0, final_speed
        else:
            return 0, min(abs(torque), MAX_BRAKE_TORQUE), final_speed

    def get_acceleration(self, delta_v, delta_t):
        accel = delta_v / delta_t
        return min(self.accel_limit, accel) if accel > 0 else max(self.decel_limit, accel)
