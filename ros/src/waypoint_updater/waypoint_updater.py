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

LOOKAHEAD_WPS = 30  # Number of waypoints will be published.


def dist(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        '''
        Subscribe to topics:
        - /current_pose: Current position (Determine immediate next waypoint)
        - /base_waypoints: Fixed world waypoints
        - /traffic_waypoint: Next stop line position (Deceleration)
        '''
        self.base_wps_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.last_wp_idx = 0
        self.base_wps = None
        self.wp_dist = None
        self.current_pose = None
        self.red_light_position = None
        self.stop_line_wp = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()

            # Skip this round if information is not ready
            if (self.base_wps is None) or (self.current_pose is None):
                continue

            # Cache references to ensure targets are not changed during execution
            base_wps = self.base_wps
            wp_dist = self.wp_dist
            wp_num = len(base_wps)
            curr_pos = self.current_pose.position

            # Look for immediate next waypoint
            # We assume the waypoints are sorted according to the order by
            # which the car is expected to go through.
            opt_idx_f, min_dist_f = self.search_immediate_next_waypoint(curr_pos, self.last_wp_idx, True)
            opt_idx_b, min_dist_b = self.search_immediate_next_waypoint(curr_pos, self.last_wp_idx, False)
            self.last_wp_idx = opt_idx_f if min_dist_f < min_dist_b else opt_idx_b
        
            # Construct waypoints for the vehicle to follow
            waypoints = []  # Contains waypoints to be published
            wp_d = []       # Contains distance between previous and current waypoint
            max_v = 0.
            for i in xrange(LOOKAHEAD_WPS):
                idx = (self.last_wp_idx + i) % wp_num  # for continuing the lap
                wp = copy.deepcopy(base_wps[idx])  # Ensures the original object is not modified
                v = wp.twist.twist.linear.x
                waypoints.append(wp)
                if v > max_v: max_v = v
                wp_d.append(wp_dist[idx])
            wp_d[0] = dist(curr_pos, waypoints[0].pose.pose.position)

            # Decelerates according to upcoming stop line position
            rospy.loginfo('### stop_line_wp: %s', self.stop_line_wp)
            if self.stop_line_wp >= 0:
                # Get deceleration limit to determine achievable speed, bounded at 1
                max_decel = min(abs(rospy.get_param('/dbw_node/decel_limit', -1.)), 1.)
                v = 0.
                idx = self.stop_line_wp

                # Traceback from the point where we need the vehicle to be stopped completely
                # A number of buffering waypoints are added to account for potential overshoots
                zero_count = 4
                while idx != (self.last_wp_idx-1) % wp_num:
                    wp_idx = (idx - self.last_wp_idx) % wp_num
                    if wp_idx < LOOKAHEAD_WPS:
                        for fw_idx in xrange(wp_idx, LOOKAHEAD_WPS):
                            fw_wp = waypoints[fw_idx]
                            if fw_wp.twist.twist.linear.x > v:
                                fw_wp.twist.twist.linear.x = v
                            else:
                                break
                    if v >= max_v: break
                    idx = (idx - 1) % wp_num
                    if zero_count > 0:
                        zero_count -= 1
                    else:
                        v = math.sqrt(v*v + 2*max_decel*wp_dist[idx])

            rospy.loginfo('#### target_vel: %s', [wp.twist.twist.linear.x for wp in waypoints])

            # Publish waypoints to /final_waypoints
            final_waypoints = Lane()
            final_waypoints.header.stamp = rospy.Time.now()
            final_waypoints.waypoints = waypoints
            self.final_waypoints_pub.publish(final_waypoints)

    def search_immediate_next_waypoint(self, curr_pos, last_idx, forward):
        '''
        Searches for the immediate next waypoint of the vehicle.

        Args:
            curr_pos (Position): Current position
            last_idx (int): Index of `self.base_wps` to search from
            forward (bool): Flag to indicate a forward (True) or backward (False) search

        Returns:
            int: Index of `self.base_wps` of the vehicle's immediate next waypoint
        '''
        # Cache references
        base_wps = self.base_wps
        wp_num = len(base_wps)
        wp_dist = self.wp_dist

        # Search for local minimum in the specified direction (forward or backward)
        min_dist = float('inf')
        opt_idx = 0
        inc = 1 if forward else -1
        prev_pos = base_wps[(last_idx - inc) % wp_num].pose.pose.position
        prev_dist = dist(prev_pos, curr_pos)
        for i in xrange(wp_num):
            idx = (last_idx + i*inc) % wp_num
            wp_pos = base_wps[idx].pose.pose.position
            curr_dist = dist(wp_pos, curr_pos)
            d = prev_dist + curr_dist
            if d < min_dist:
                min_dist = d
                opt_idx = idx
            else:
                return opt_idx if forward else (opt_idx+1) % wp_num, min_dist
            prev_dist = curr_dist
        return opt_idx if forward else (opt_idx+1) % wp_num, min_dist

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        
    def waypoints_cb(self, waypoints):
        wps = waypoints.waypoints
        # Preprocess distance of each waypoint from its predecessor
        wp_dist = [dist(wps[i].pose.pose.position, wps[i-1].pose.pose.position) for i in xrange(len(wps))]
        self.wp_dist = wp_dist
        self.base_wps = wps
        self.base_wps_sub.unregister()

    def traffic_cb(self, msg):
        self.stop_line_wp = msg.data

    def obstacle_cb(self, msg):
        pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
