#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

import math

import functools, copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


def dist(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.last_wp_idx = 0
        self.base_wps = None
        self.wp_dist = None
        self.current_pose = None
        self.red_light_position = None
        self.current_velocity = 0
        self.red_light_wp = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()
            if (self.base_wps is None) or (self.current_pose is None):
                continue
            base_wps = self.base_wps
            wp_dist = self.wp_dist
            curr_pos = self.current_pose.position
            curr_ort = self.current_pose.orientation

            wp_num = len(base_wps)
            prev_pos = base_wps[self.last_wp_idx-1].pose.pose.position
            prev_dist = dist(prev_pos, curr_pos)

            # Look for immediate next waypoint
            # We assume the waypoints are sorted according to the order by
            # which the car is expected to go through.
            for i in range(wp_num):
                idx = (self.last_wp_idx + i) % wp_num
                wp_pos = base_wps[idx].pose.pose.position
                seg_dist = dist(wp_pos, prev_pos)
                wp_dist = dist(wp_pos, curr_pos)

                if wp_dist <= seg_dist and prev_dist <= seg_dist:
                   self.last_wp_idx = idx
                   break
                prev_pos = wp_pos
                prev_dist = wp_dist
        
            # TODO: Need modifications to take care the traffic light scenario
            # Construct waypoints for the vehicle to follow
            waypoints = []
            wp_d = []
            for i in range(LOOKAHEAD_WPS):
                idx = (self.last_wp_idx + i) % wp_num  # for continuing the lap
                wp = copy.deepcopy(base_wps[idx])
                waypoints.append(wp)
                wp_d.append(wp_dist[idx])
            wp_d[0] = dist(curr_pos, waypoints[0].pose.pose.position)


            # Get acceleration limit to determine achievable speed
            accel_limit = rospy.get_param('/dbw_node/accel_lmit', 1.)
            v = self.current_velocity
            max_v = 0.
            for wp, dist in zip(waypoints, wp_d):
                v = min(wp.twist.twist.linear.x, math.sqrt(v*v + 2*accel_limit*dist)
                wp.twist.twist.linear.x = v
                if v > max_v: max_v = v

            # Get deceleration limit to determine achievable speed
            if self.red_light_wp >= 0:
                max_decel = min(abs(rospy.get_param('/dbw_node/decel_limit', -1.)), 1.)
                v = 0.
                idx = self.red_light_wp
                max_zero_count = 6
                count = 0
                while idx != self.last_wp_idx-1:
                    if count < max_zero_count:
                        count += 1
                    else:
                        v = math.sqrt(v*v + 2*max_decel*wp_dist[(idx+1) % wp_num])
                    wp_idx = (idx - self.last_wp_idx) % wp_num
                    if wp_idx < LOOKAHEAD_WPS:
                        wp = waypoints[wp_idx]
                        if v < wp.twist.twist.linear.x:
                            wp.twist.twist.linear.x = v
                        else:
                            break
                    if v >= max_v: break
                    idx = (idx - 1) % wp_num

            # Publish waypoints to /final_waypoints
            final_waypoints = Lane()
            final_waypoints.header.stamp = rospy.Time.now()
            final_waypoints.waypoints = waypoints
            self.final_waypoints_pub.publish(final_waypoints)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        wps = waypoints.waypoints
        wp_dist = [dist(wps[i].pose.pose.position, wps[i-1].pose.pose.position) for i in range(len(wps))]
        self.wp_dist = wp_dist
        self.base_wps = wps

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_wp = msg.data


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
