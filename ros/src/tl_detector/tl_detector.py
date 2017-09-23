#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from deep_detector.deep_detector import DeepDetector
import tf
import cv2
import yaml

import numpy as np
import bisect as bs

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_lines = None

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        model_path = rospy.get_param('~model_path')
        self.deep_detector = DeepDetector(model_path)
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        stop_lines = [self.get_closest_waypoint(p) for p in self.config['stop_line_positions']]
        stop_lines.sort()
        self.stop_lines = stop_lines
        self.sub2.unregister()

    def traffic_cb(self, msg):
        if self.waypoints is None:
            return
        wp_num = len(self.waypoints)
        lights = [(self.get_closest_waypoint(l.pose.pose), l) for l in msg.lights]
        lights.sort()
        self.lights = lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose/list): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        # Naive approach
        if isinstance(pose, Pose):
            pose = [pose.position.x, pose.position.y]
        def d2(p1, p2):
            dx = p1[0] - p2.position.x
            dy = p1[1] - p2.position.y
            return dx*dx + dy*dy

        if self.waypoints is None:
            return 0
        waypoints = self.waypoints

        closest_idx = 0
        min_dist = float('inf')
        for idx, wp in enumerate(waypoints):
            d = d2(pose, wp.pose.pose)
            if d < min_dist:
                min_dist = d
                closest_idx = idx
        return closest_idx

    def get_light_state(self):
        if (not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "passthrough")
        return self.deep_detector.get_light_state(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        if not all([self.pose, self.lights, self.waypoints]):
            return -1, TrafficLight.UNKNOWN
        car_position = self.get_closest_waypoint(self.pose.pose)

        if (self.lights is None) or (self.waypoints is None) or (self.stop_lines is None):
            return -1, TrafficLight.UNKNOWN

        lights = self.lights
        max_wp = lights[-1][0]
        min_wp = lights[0][0]
        light_wp = len(self.waypoints)
        if car_position > max_wp:
            light_wp, light = lights[0]
        else:
            for wp_idx, tl in lights:
                if wp_idx >= car_position:
                    light_wp, light = wp_idx, tl
                    break

        if light:
            state = self.get_light_state()
            stop_line_idx = bs.bisect_right(self.stop_lines, light_wp)-1
            return self.stop_lines[stop_line_idx], state
        return -1, TrafficLight.UNKNOWN

    def close(self):
        self.deep_detector.close()

if __name__ == '__main__':
    try:
        tl_detector = TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
    finally:
        tl_detector.close()
