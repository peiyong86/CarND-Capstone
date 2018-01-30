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
import copy
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
# BASE_VELOCITY = 11.111

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_waypoints = Lane()
        # self.traffic_waypoint = Int32()
        self.waypoints_available = False # check whether this nodes has the initial pose of the vehicle

        self.compute_distance = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

        #self.current_waypoint_index = 0
        self.current_red_line_waypoint = -1 #753 #753 is the second traffic light. this should be 0 in the final submission.

        #self.start_to_brake_distance = 30 #If the distance between the red traffic light and car is less than this value, start to brake
        self.current_velocity = TwistStamped()
        self.current_velocity.twist.linear.x = 0.0
        # self.target_v = 11.0 # this should be initialized from base waypoints
        self.accel = 3.0 # any requirement?
        self.decel = 0.4 #3.0 # any requirement?



        rospy.Subscriber('/current_velocity', TwistStamped, 
            self.current_velocity_cb, queue_size = 1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_array_cb) # this should be removed for the final submission. Debug purpose.
        # rospy.Susscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below


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
        if self.waypoints_available is True:
            base_waypoint_len = len(self.base_waypoints.waypoints)
            
            current_waypoint_index = self.get_idx(
                self.base_waypoints.waypoints, msg)
            #print("cur idx:", current_waypoint_index)
            # define Lane message and add LOOKAHEAD_WPS waypoints ahead of the car
            final_waypoints = Lane()
            final_waypoints.header = current_pose.header

            #======================================================
            # Use a red light waypoint to modify the velocities of
            # the final waypoints
            # The basic physical formula for speed calculation is:
            # V2^2 - V1^2 = 2 * a * s, while V1, V2 are the speed at
            # two points, a is the accel or decel, s is displacement
            #======================================================
             
            cur_v = self.current_velocity.twist.linear.x
            points = [0]
            brake = False
            decel = 0
            stop_bias = 0
            current_red_line_waypoint = self.current_red_line_waypoint
            if current_red_line_waypoint != -1:
                current_red_line_waypoint = current_red_line_waypoint-stop_bias

                safe_brake_dist = cur_v * cur_v / (2 * self.decel)

                distance = 0
                idx = current_red_line_waypoint
                while distance < safe_brake_dist: #safe_brake_dist:
                    pre_idx = (idx + base_waypoint_len - 1) % base_waypoint_len
                    distance += self.compute_distance(
                        self.base_waypoints.waypoints[pre_idx].pose.pose.position,
                        self.base_waypoints.waypoints[idx].pose.pose.position)
                    points.append(distance)
                    idx = pre_idx
                # now idx is the where the car should brake
                if current_waypoint_index < idx or \
                    (current_waypoint_index > idx + base_waypoint_len / 2):
                    #no need to brake now
                    pass
                else:
                    # here, we are already ahead of the safe brake idx, 
                    # need to brake
                    brake = True
                    if (current_waypoint_index >= current_red_line_waypoint):
                        # it's possible that the car already stops, but just a little
                        # ahead of the red_line, in this case, speed will be 0 as 
                        # below
                        pass
                    else:
                        
                        idx = (current_red_line_waypoint + base_waypoint_len - 
                            current_waypoint_index ) % base_waypoint_len
                        decel = cur_v * cur_v / (2 * points[idx])
                    #if (abs(current_waypoint_index - current_red_line_waypoint) < 20):
                    #    print("cur idx:", current_waypoint_index, cur_v)

            num = 0
            pre_v = cur_v
            pre_position = current_pose.pose.position
            while num < LOOKAHEAD_WPS+1:
                idx = (current_waypoint_index + num) % base_waypoint_len
                if brake:
                    if idx >= current_red_line_waypoint:
                        v = 0
                    else:
                        distance = points[(current_red_line_waypoint +
                            base_waypoint_len - idx) % base_waypoint_len] 
                        v = math.sqrt(2 * decel * distance) 
                        #if (current_red_line_waypoint - idx < 3):
                        #    print(idx, v)
                else:
                    self.target_v = self.base_waypoints.waypoints[idx].twist.twist.linear.x
                    if (pre_v >= self.target_v):
                        v = self.target_v
                    else:
                        s = self.compute_distance(pre_position,
                            self.base_waypoints.waypoints[idx].pose.pose.position)
                        v = min(math.sqrt(pre_v * pre_v + 2 * self.accel * s),
                            self.target_v)
                        pre_v = v
                #print(idx, v,)
                waypoint = copy.deepcopy(self.base_waypoints.waypoints[idx])
                waypoint.twist.twist.linear.x = v

                # self.set_waypoint_velocity(self.base_waypoints.waypoints, idx, v)
                # final_waypoints.waypoints.append(self.base_waypoints.waypoints[idx])
                if num>=1:
                    final_waypoints.waypoints.append(waypoint)
                # if num<10:
                #     print('car_in{}, cur_v={}, brake={}, num={},v={}'.format(current_waypoint_index,cur_v, brake, num,v))

                num += 1

            # publish waypoints
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

        self.waypoints_available = True
        self.base_waypoints = waypoints
        # self.target_v should be set here
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
            if msg.data!=-1:
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

    def get_idx(self, waypoints, msg):
        idx_x = -1
        idx_y = -1
        delta_x = 0
        delta_y = 0
        total = len(waypoints)

        for i in range(total):
            x1 = waypoints[i].pose.pose.position.x
            y1 = waypoints[i].pose.pose.position.y
            x2 = waypoints[(i + 1) % total].pose.pose.position.x
            y2 = waypoints[(i + 1) % total].pose.pose.position.y
            x = msg.pose.position.x
            y = msg.pose.position.y
            if idx_x == -1 and (x - x1) * (x2 - x) >= 0 and abs(y - y1) < 10:
                idx_x = i;
            if idx_y == -1 and (y - y1) * (y2 - y) >= 0 and abs(x - x1) < 10:
                idx_y = i;
            if idx_x != -1 and idx_y != -1:
                break
        if idx_x == -1 and idx_y == -1:
            print("fatal error, can't locat the car!")
        if idx_x != -1:
            delta_x = abs(waypoints[(idx_x + 1) % total].pose.pose.position.x -
                waypoints[idx_x].pose.pose.position.x)
        if idx_y != -1:
            delta_y = abs(waypoints[(idx_y + 1) % total].pose.pose.position.y -
                waypoints[idx_y].pose.pose.position.y)
        if delta_x > delta_y:
            return idx_x + 1
        else:
            return idx_y + 1

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
