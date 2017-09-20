
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import math
import rospy
import numpy as np

from speed_controller import SpeedController
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                       wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.speed_controller = SpeedController(total_mass, wheel_radius, brake_deadband,
                                                accel_limit, decel_limit)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.,
                                            max_lat_accel, max_steer_angle)
        kp, ki, kd = np.array([0.63, 0.003, 2.]) * np.pi / 180. * steer_ratio
        self.correction_pid = PID(kp, ki, kd, mn=-math.pi/2., mx=math.pi/2.)
        self.angle_filter = LowPassFilter(1., 1.)
        self.steer_filter = LowPassFilter(1., 1.)
        self.timestamp = 0
        while not self.timestamp:
            self.timestamp = rospy.Time.now().to_sec()

    def control(self, linear_velocity, angular_velocity, current_velocity, enable_dbw, cte):
        if not enable_dbw:
            # Reset PID
            self.correction_pid.reset()
            self.speed_controller.reset()
            return 0, 0, False

        # t = rospy.Time.now().to_sec()
        # delta_t = t - self.timestamp
        # self.timestamp = t

        # Calculate throttle, and under which the linear velocity after 2s
        ctrl, final_velocity, is_throttle = self.speed_controller.get_control(linear_velocity,
                                                                              current_velocity,
                                                                              2.)

        if current_velocity >= .5:
            correction = self.correction_pid.step(cte, delta_t)
            angular_velocity = self.angle_filter.filt(angular_velocity)
            steer = self.yaw_controller.get_steering(final_velocity, angular_velocity, current_velocity) + correction
            steer = self.steer_filter.filt(steer)
        else:
            steer = self.steer_filter.get()
        
        return ctrl, steer, is_throttle

