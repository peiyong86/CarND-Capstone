#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math

#added by us
import numpy as np
import tf #This is tf in ros NOT tensorflow!

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
BASE_VELOCITY = 11.111

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_array_cb) # this should be removed for the final submission. Debug purpose.
        # rospy.Susscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = Lane()
        # self.traffic_waypoint = Int32()
        self.waypoints_avaialbe = False # check whether this nodes has the initial pose of the vehicle

        self.compute_distance = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

        self.current_waypoint_index = 0
        self.current_red_line_waypoint = -1 #753 is the second traffic light. this should be 0 in the final submission.

        self.start_to_brake_distance = 30 #If the distance between the red traffic light and car is less than this value, start to brake
        self.current_velocity = TwistStamped()
        self.current_velocity.twist.linear.x = 11.0

        rospy.spin()
    def current_velocity_cb(self, msg):
        '''
        geometry_msgs/TwistStamped
            std_msgs/Header header
            geometry_msgs/Twist twist
                geometry_msgs/Vector3 linear
                    float64 x
                    float64 y
                    float64 z
                geometry_msgs/Vector3 angular
                    float64 x
                    float64 y
                    float64 z

        :param msg:
        :return:
        '''

        self.current_velocity = msg


    def pose_cb(self, msg):
        '''

        :param msg:
        :return:

        '''

        current_pose = msg
        # rospy.loginfo(current_pose.pose.position.x, current_pose.pose.position.y)
        if self.waypoints_avaialbe is True:
            #====================================
            # FIND THE CLOSEST WAYPOINT
            # ====================================
            # compute distances between the current position and all waypoints
            base_waypoint_length = len(self.base_waypoints.waypoints)
            distances = [self.compute_distance(current_pose.pose.position, self.base_waypoints.waypoints[i].pose.pose.position) for i in range(base_waypoint_length)]
            # find the closest base waypoint by computing argmin
            closest_index = np.argmin(distances)

            #====================================================
            # Find the waypoint ahead of the car by transforming
            # the waypoints in the world frame to the car frame
            #====================================================
            quaternion = (
            current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z,
            current_pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            for i in range(3):
                temp_index = (closest_index - 1+i)%base_waypoint_length
                x = self.base_waypoints.waypoints[temp_index].pose.pose.position.x - current_pose.pose.position.x
                y = self.base_waypoints.waypoints[temp_index].pose.pose.position.y - current_pose.pose.position.y

                x_in_car_frame =  math.cos(yaw) * x + math.sin(yaw) * y
                y_in_car_frame = -math.sin(yaw) * x + math.cos(yaw) * y

                if x_in_car_frame >= 0:
                    closest_index = temp_index
                    # print("closest_index={}, i ={}".format(closest_index, i))
                    break

            self.current_waypoint_index = closest_index
            # define Lane message and add LOOKAHEAD_WPS waypoints ahead of the car
            final_waypoints = Lane()
            final_waypoints.header = current_pose.header


            #======================================================
            # Use a red light waypoint to modify the velocities of
            # the final waypoints
            # TODO: how to change the velocity?
            # I linearly reduced the velocities now. We can fit them
            # to other functions.
            #======================================================
            # rospy.loginfo("current_index={}, red_light_index={}, x={}, y={}".format(self.current_waypoint_index, self.current_red_line_waypoint, current_pose.pose.position.x, current_pose.pose.position.y))
            if self.current_red_line_waypoint!=-1:
                dist_between_car_and_light=self.distance(self.base_waypoints.waypoints, self.current_waypoint_index, self.current_red_line_waypoint)

                if dist_between_car_and_light<=self.start_to_brake_distance:

                    waypoints_between_car_and_light = []
                    if self.current_waypoint_index < self.current_red_line_waypoint:
                        waypoints_between_car_and_light = range(self.current_waypoint_index, self.current_red_line_waypoint)
                    else : #I consider the case, the car goes back to the starting point. I don't know the end of the road. I assume it comes back to the start.
                        waypoints_between_car_and_light = range(self.current_waypoint_index, base_waypoint_length)
                        waypoints_between_car_and_light = waypoints_between_car_and_light + range(0, self.current_red_line_waypoint)

                    # original_velocity = self.base_waypoints.waypoints[self.current_red_line_waypoint].twist.twist.linear.x

                    for i, waypoint_i in enumerate(reversed(waypoints_between_car_and_light)):
                    # for i in range(LOOKAHEAD_WPS):

                        if(waypoints_between_car_and_light>=6):
                            temp_velocity = max(self.current_velocity.twist.linear.x, 5.0)  # we need some thrust. If current velocity is too low, it cannot reach the stop line.
                        else:
                            temp_velocity = self.current_velocity.twist.linear.x
                        self.base_waypoints.waypoints[waypoint_i].twist.twist.linear.x = max(0, 0  + (i-3) * (temp_velocity)/len(waypoints_between_car_and_light))
                        # final_waypoints.waypoints[i].twist.twist.linear.x = max(11-i * 11.0/len(waypoints_between_car_and_light),0)
                    # consider some safe margin
                    for j in range(10):
                        self.base_waypoints.waypoints[(self.current_red_line_waypoint + j+1)%base_waypoint_length].twist.twist.linear.x = 0
                else:
                    for i in range(LOOKAHEAD_WPS):
                        self.base_waypoints.waypoints[(self.current_waypoint_index -1 + i)%base_waypoint_length].twist.twist.linear.x = BASE_VELOCITY
            else:
                for i in range(LOOKAHEAD_WPS):
                    self.base_waypoints.waypoints[(self.current_waypoint_index -1 + i)%base_waypoint_length].twist.twist.linear.x = BASE_VELOCITY
            #======================================================
            # Publish the final waypoints
            #======================================================
            for i in range(LOOKAHEAD_WPS):
                new_waypoint = self.base_waypoints.waypoints[(closest_index + i)%base_waypoint_length]
                if i==0:
                    final_waypoints.waypoints =[new_waypoint]
                else:
                    final_waypoints.waypoints.append(new_waypoint)

            # publish final waypoints here
            self.final_waypoints_pub.publish(final_waypoints)

    def traffic_light_array_cb(self, msg):
        self.traffic_lights = msg

    def waypoints_cb(self, waypoints):

        '''
        :param waypoints:
        :return:

        # TODO: Implement
        # /base_waypoints is published by waypoint loader
        '''

        self.waypoints_avaialbe = True
        self.base_waypoints = waypoints


    def traffic_cb(self, msg):
        '''

        :param msg:
        :return:

        # TODO: Callback for /traffic_waypoint message. Implement
        # /traffic_waypoint is published by traffic light detection node
        # msg.data is the waypoint that is the nearest to an upcoming red light's stop line.
        # For example, assume that msg.data = 12. Then, the velocity at waypoint[12] should be zero.
        '''

        self.traffic_waypoint = msg  #msg.data
#        rospy.loginfo("data={}".format(msg.data))

        if self.traffic_lights:
 #           rospy.loginfo("current_light_index={}, more info={}".format(msg.data,self.traffic_lights.lights[self.traffic_waypoint.data]))
            rospy.loginfo("current_light_index={}".format(msg.data))

        self.current_red_line_waypoint = msg.data

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
