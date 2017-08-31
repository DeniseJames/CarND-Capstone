#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
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

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.last_wp_idx = 0
        self.base_wps = None
        self.red_light_position = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if self.base_wps is None:
            return
        base_wps = list(self.base_wps)
        curr_pos = msg.pose.position
        orientation = msg.pose.orientation

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
        for i in range(LOOKAHEAD_WPS):
            idx = (self.last_wp_idx + i) % wp_num
            wp = copy.deepcopy(base_wps[idx])
            waypoints.append(wp)
        
        # Publish waypoints to /final_waypoints
        final_waypoints = Lane()
        final_waypoints.header.stamp = rospy.Time.now()
        final_waypoints.waypoints = waypoints
        self.final_waypoints_pub.publish(final_waypoints)
        
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_wps = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
