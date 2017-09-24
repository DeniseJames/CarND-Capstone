
MAX_ACCEL_TORQUE = 2000.

import math
from pid import PID
from lowpass import LowPassFilter


class SpeedController(object):
    def __init__(self, total_mass, wheel_radius, brake_deadband, accel_limit, decel_limit):
        self.total_mass = total_mass
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.accel_lowpass_filter = LowPassFilter(2., 1.)
        
        self.friction_coeff = 0.018641  # For throttle against friction

    def get_control(self, target_speed, current_speed, delta_t):
        """
        Computes linear controls

        target_speed: target speed, not necessarily to be achievable
        current_speed: current speed
        delta_t: lookahead time for prediction
        """

        delta_v = target_speed - current_speed
        accel = self.get_acceleration(delta_v, delta_t)
        accel = self.accel_lowpass_filter.filt(accel)
        final_speed = current_speed + accel * delta_t         # v = u + a*t
        torque = self.total_mass * accel * self.wheel_radius  # T = F*r = m*a*r
        friction = self.friction_coeff * current_speed  # Throttle required to keep current velocity

        throttle = min(torque / MAX_ACCEL_TORQUE + friction, 1.)
        if throttle > 0: return throttle, final_speed, True

        if abs(accel) < self.brake_deadband: return 0., final_speed, False
        return max(abs(torque)-friction*MAX_ACCEL_TORQUE, 0.), final_speed, False

    def reset(self):
        pass

    def get_acceleration(self, delta_v, delta_t):
        accel = delta_v / delta_t
        return min(self.accel_limit, accel) if accel > 0 else max(self.decel_limit, accel)
        
