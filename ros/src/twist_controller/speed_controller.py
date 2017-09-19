
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
        self.pid = PID(kp=1., ki=0.005, kd=1.5, mn=decel_limit, mx=accel_limit)
        self.lowpass_filter = LowPassFilter(1., 1.)
        self.friction_coeff = 0.018641

    def get_control(self, target_speed, current_speed, delta_t):
        """
        Computes linear controls

        target_speed: target speed, not necessarily to be achievable
        current_speed: current speed
        delta_t: lookahead time for prediction
        """

        delta_v = target_speed - current_speed
        # accel = self.pid.step(delta_v, delta_t)
        accel = self.get_acceleration(delta_v, delta_t)
        accel = self.lowpass_filter.filt(accel)
        final_speed = current_speed + accel * delta_t
        torque = self.total_mass * accel * self.wheel_radius

        friction = self.friction_coeff * current_speed
        
        if torque >= 0.:
            return min(torque / MAX_ACCEL_TORQUE + friction, 1.), final_speed, True
        decel= abs(accel)
        if decel < self.brake_deadband:
            return 0., current_speed, False
        brake = abs(torque)
        return brake, final_speed, False

    def reset(self):
        self.pid.reset()

    def get_acceleration(self, delta_v, delta_t):
        accel = delta_v / delta_t
        return min(self.accel_limit, accel) if accel > 0 else max(self.decel_limit, accel)
        
