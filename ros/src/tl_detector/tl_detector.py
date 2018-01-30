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
import os

STATE_COUNT_THRESHOLD = 3

GLOBAL_I = 0

class TLDetector(object):
    def __init__(self):
        # rospy.init_node('tl_detector')
        rospy.loginfo("TL detector node inited.")

        self.has_image = False
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)#, buff_size=5000000)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()


        # rospy.spin()

    def run_classification(self):
        if self.has_image:
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


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # rospy.loginfo("Received incoming msg.")
        self.has_image = True
        self.camera_image = msg
        # light_wp, state = self.process_traffic_lights()
        #
        # '''
        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        # of times till we start using it. Otherwise the previous stable state is
        # used.
        # '''
        # if self.state != state:
        #     self.state_count = 0
        #     self.state = state
        # elif self.state_count >= STATE_COUNT_THRESHOLD:
        #     self.last_state = self.state
        #     light_wp = light_wp if state == TrafficLight.RED else -1
        #     self.last_wp = light_wp
        #     self.upcoming_red_light_pub.publish(Int32(light_wp))
        # else:
        #     self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        # self.state_count += 1

    @staticmethod
    def calculate_dist(x1, y1, x2, y2):
        """Remove sqrt part to speed up calculation, while keep the order.
        """
        return (x1 - x2)**2 + (y1 - y2)**2

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # rospy.loginfo("pose.position {}".format(pose.position))
        px = pose.position.x
        py = pose.position.y

        mindist = None
        minindex = None

        for i, point in enumerate(self.waypoints.waypoints):
            wx = point.pose.pose.position.x
            wy = point.pose.pose.position.y
            dist = self.calculate_dist(px, py, wx, wy)
            if not mindist:
                mindist = dist
                minindex = i
            if dist < mindist:
                mindist = dist
                minindex = i
        return minindex

    def get_light_state(self, light, dist):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # rospy.loginfo("Get light state.")
        if(not self.has_image):
            self.prev_light_loc = None
            # rospy.loginfo("No image here.")
            return False

        if dist > 3000:
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # save some image
        # global GLOBAL_I
        # cv2.imwrite('temp_{:0>4}_{:0>4}.jpg'.format(GLOBAL_I, dist), cv_image)
        # GLOBAL_I += 1
        # rospy.loginfo("image saved at {}".format(os.getcwd()))

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        # rospy.loginfo("Finding closest waypoint to car pos.")
        # rospy.loginfo("Car pose is {}".format(self.pose))
        if(self.pose and self.waypoints):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN

        # find the closest visible traffic light (if one exists)
        # find the stop line that closest to car
        closest_dist = None
        closest_trafficlight_waypoint = None
        closest_stop_line_waypoint = None
        for stop_line_position in stop_line_positions:
            dist = self.calculate_dist(stop_line_position[0], stop_line_position[1],
                                       self.pose.pose.position.x, self.pose.pose.position.y)

            # find the stop line's corresponding waypoint
            stopline_pos = Pose()
            stopline_pos.position.x = stop_line_position[0]
            stopline_pos.position.y = stop_line_position[1]
            stop_line_waypoint = self.get_closest_waypoint(stopline_pos)

            if stop_line_waypoint > car_position:
                if not closest_dist:
                    closest_dist = dist
                    closest_trafficlight_waypoint = stop_line_waypoint
                    closest_stop_line_waypoint = stop_line_waypoint
                if dist < closest_dist:
                    closest_dist = dist
                    closest_trafficlight_waypoint = stop_line_waypoint
                    closest_stop_line_waypoint = stop_line_waypoint

        # rospy.loginfo('distance to next traffic light is {}, index={}'.format(closest_dist, closest_stop_line_waypoint))

        index_2_label = {TrafficLight.UNKNOWN:'unknow',
                         TrafficLight.GREEN:'green',
                         TrafficLight.YELLOW:'yellow',
                         TrafficLight.RED:'red'}

        if not closest_trafficlight_waypoint:
            return -1, TrafficLight.UNKNOWN
        else:
            light = closest_trafficlight_waypoint
            state = self.get_light_state(light, closest_dist)
            rospy.loginfo("Light {} detected.".format(index_2_label[state]))
            # return stop_line_waypoint, state
            return closest_stop_line_waypoint, state
        #self.waypoints = None
        #return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        rospy.init_node('tl_detector')
        tl=TLDetector()
        rate = rospy.Rate(30) # 10hz
        while not rospy.is_shutdown():
            tl.run_classification()
            rate.sleep()


    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')