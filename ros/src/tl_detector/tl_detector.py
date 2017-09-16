#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

import numpy as np

STATE_COUNT_THRESHOLD = 3
OFFSET = -30


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        #### For saving camera images
        # sub6 = rospy.Subscriber('/image_color', Image, self.image_save_cb, queue_size=None)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        ### For saving camera images
        # self.save_count = 0
        # self.loop()
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        if self.waypoints is None:
            return
        wp_num = len(self.waypoints)
        lights = [((self.get_closest_waypoint(l.pose.pose)+OFFSET)%wp_num, l) for l in msg.lights]
        lights.sort()
        self.lights = lights

    #### For saving camera images
    # def image_save_cb(self, msg):
    #     self.camera_image = msg

    #### For saving camera images
    # def loop(self):
    #     rate = rospy.Rate(5)
    #     while not rospy.is_shutdown():
    #         if self.camera_image is None:
    #             rate.sleep()
    #             continue
    #         self.camera_image.encoding = 'rgb8'
    #         cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
    #         cv2.imwrite('images/%06d.png' % self.save_count, cv_image)
    #         self.save_count += 1
    #         rate.sleep()
        

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        # Naive approach
        def d2(p1, p2):
            dx = p1.position.x - p2.position.x
            dy = p1.position.y - p2.position.y
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

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']

        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        q = (rot.x, rot.y, rot.z, rot.w)
        row, pitch, yaw = tf.transformations.euler_from_quaternion(q)

        cx, sx = np.cos(row), np.sin(row)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])

        cy, sy = np.cos(pitch), np.sin(pitch)
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])

        cz, sz = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

        R = np.dot(Rx, np.dot(Rx, Rz))

        rvec = cv2.Rodrigues(R)
        tvec = np.array([trans.x, trans.y, trans.z])
        cam_mat = np.array([[fx, 0, 0], [0, fy, 0], [0, 0, 1]])

        p_world = np.array([point_in_world.x, point_in_world.y, point_in_world.z])

        p_outs, _ = cv2.projectPoints([p_world], rvec, tvec, cam_mat, None)
        x, y = p_outs.flatten()
        x = int(round(x + image_width/2))
        y = int(round(y + image_height/2))
        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image
        # Need to define a region and an input size for the model

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = self.config['light_positions']
        if not all([self.pose, self.lights, self.waypoints]):
            return -1, TrafficLight.UNKNOWN
        car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if (self.lights is None) or (self.waypoints is None):
            return -1, TrafficLight.UNKNOWN

        lights = self.lights
        max_wp = lights[-1][0]
        min_wp = lights[0][0]
        light_wp = len(self.waypoints)
        for wp_idx, tl in lights:
            if ((wp_idx >= car_position) and (wp_idx < light_wp)) \
                or ((wp_idx == min_wp) and (car_position > max_wp)):
                light_wp = wp_idx
                light = tl

        ############################################################
        # Here we use the data given by "/vehicle/traffic_lights" for development purpose.
        # Need to change back to config file before submission
        if light:
            return light_wp, light.state
        ############################################################

        # if light:
        #     state = self.get_light_state(light)
        #     return light_wp, state
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
