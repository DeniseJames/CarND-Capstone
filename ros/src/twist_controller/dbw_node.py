#!/usr/bin/env python

import rospy, tf
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane

import math
import numpy as np

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create controller
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                                     accel_limit, wheel_radius, wheel_base, steer_ratio,
                                     max_lat_accel, max_steer_angle)

        # Create placeholders for subscription data
        self.target = None              # tuple: (linear_velocity, angular_velocity)
        self.curr_v = 0.                # double
        self.dbw_enabled = False        # bool
        self.curr_coord = None          # tuple: (x, y)
        self.curr_yaw = 0.              # double
        self.final_waypoints = None     # list of waypoints
        self.max_steer = max_steer_angle

        """
        Subscribe to topics:
        - /twist_cmd: Target linear and angular velocity (Determine controls)
        - /current_velocity: Current velocity (Determine controls)
        - /vehicle/dbw_enabled: Flag indicates autonomous or manual mode
        - /current_pose: Current position (Determine CTE for steering PID)
        - /final_waypoints: Upcoming waypoints to follow
        """
        self.twist_cmd_sub = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        self.current_velocity_sub = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        self.dbw_enabled_sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb, queue_size=1)
        self.final_waypoints_sub = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Skip this round if not everything is ready
            if (self.target is None) or (self.curr_coord is None) or (self.final_waypoints is None):
                rate.sleep()
                continue

            # Get controls
            cte = self.get_cte()
            ctrl, steer, is_throttle = self.controller.control(self.target[0],
                                                               self.target[1],
                                                               self.curr_v,
                                                               self.dbw_enabled,
                                                               cte)

            steer = min(max(-self.max_steer, steer), self.max_steer)  # Crop steering angle
            if self.dbw_enabled:
                if is_throttle: rospy.loginfo('throttle: %s, steer: %s', ctrl, steer)
                else: rospy.loginfo('brake: %s, steer: %s', ctrl, steer)
                self.publish(ctrl, steer, is_throttle)
            rate.sleep()

    def publish(self, ctrl, steer, is_throttle):
        """
        Publishes controls.

        Args:
            ctrl: Speed control, either throttle or brake
            steer: Steer control
            is_throttle: Flag indicates whether `ctrl` is throttle or brake
        """
        if is_throttle:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = ctrl  # throttle
            self.throttle_pub.publish(tcmd)
        else:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = ctrl  # brake
            self.brake_pub.publish(bcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

    def twist_cmd_cb(self, msg):
        self.target = (msg.twist.linear.x, msg.twist.angular.z)

    def current_velocity_cb(self, msg):
        self.curr_v = msg.twist.linear.x

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def current_pose_cb(self, msg):
        q = (msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z,
             msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)
        self.curr_yaw = euler[2]
        self.curr_coord = (msg.pose.position.x, msg.pose.position.y)

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints
        rospy.loginfo('#### target_vels: %s', [l.twist.twist.linear.x for l in self.final_waypoints])

    def get_cte(self):
        # Fit waypoints with polynomial or order 3 (at most), using up to 8 upcoming waypoints.
        waypoints = self.final_waypoints[:8]
        dy = waypoints[-1].pose.pose.position.y - waypoints[0].pose.pose.position.y
        dx = waypoints[-1].pose.pose.position.x - waypoints[0].pose.pose.position.x
        yaw = math.atan2(dy, dx)
        c, s = math.cos(-yaw), math.sin(-yaw)
        x0, y0 = self.curr_coord
        order = min(3, len(waypoints)-1)
        xs = []
        ys = []
        for wp in waypoints:
            x = wp.pose.pose.position.x - x0
            y = wp.pose.pose.position.y - y0
            xs.append(c*x - s*y)
            ys.append(s*x + c*y)

        # Apply Newton's method to find cte, using 5 iterations
        f = np.polyfit(xs, ys, order)
        fp = [i*f[i] for i in xrange(1, len(f))]
        fpp = [i*fp[i] for i in xrange(1, len(fp))]

        xn = 0.  # Initial guess
        for _ in xrange(5):
            f_n = np.polyval(f, xn)
            fp_n = np.polyval(fp, xn)
            fpp_n = np.polyval(fpp, xn)
            xn -= (f_n*fp_n + xn) / (fp_n*fp_n + f_n*fpp_n + 1.)

        yn = np.polyval(f, xn)
        cte = np.sqrt(xn*xn + yn*yn)
        return cte if yn > 0 else -cte


if __name__ == '__main__':
    DBWNode()
