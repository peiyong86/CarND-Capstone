#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Susscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = Lane()
        # self.traffic_waypoint = Int32()
        self.waypoints_avaialbe = False # check whether this nodes has the initial pose of the vehicle

        self.compute_distance = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

        rospy.spin()

    def pose_cb(self, msg):
        '''

        :param msg:
        :return:

        # TODO: I don't consider the velocity of the vehicle to compute the first final wayponit now. I need to revise it later. 1/2/2017
        '''

        current_pose = msg

        if self.waypoints_avaialbe is True:
            #====================================
            # FIND THE CLOSEST WAYPOINT
            # ====================================
            # compute distances between the current position and all waypoints
            base_waypoint_length = len(self.base_waypoints.waypoints)
            distances = [self.compute_distance(current_pose.pose.position, self.base_waypoints.waypoints[i].pose.pose.position) for i in range(base_waypoint_length)]
            # find the closest base waypoint by computing argmin
            closest_index = np.argmin(distances)

            #Find the waypoint ahead of the car by transfroming thw waypoints to the car frame
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

            # define Lane message and add LOOKAHEAD_WPS waypoints ahead of the car
            final_waypoints = Lane()
            final_waypoints.header = current_pose.header
            for i in range(LOOKAHEAD_WPS):
                new_waypoint = self.base_waypoints.waypoints[(closest_index + i)%base_waypoint_length]
                final_waypoints.waypoints.append(new_waypoint)

            # publish final waypoints here
            self.final_waypoints_pub.publish(final_waypoints)

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
        '''

        self.traffic_waypoint = msg  #msg.data

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
