from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy
import math
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    # def __init__(self, *args, **kwargs):
    def __init__(self, params):
        # TODO: Implement

        self.vehicle_mass = params['vehicle_mass']
        fuel_capacity = params['fuel_capacity']
        self.brake_deadband = params['brake_deadband']
        self.decel_limit = params['decel_limit']
        accel_limit = params['accel_limit']
        self.wheel_radius = params['wheel_radius']
        wheel_base = params['wheel_base']
        steer_ratio = params['steer_ratio']
        max_lat_accel = params['max_lat_accel']
        max_steer_angle = params['max_steer_angle']

        self.throttle_scaling = params['throttle_scaling']
        self.brake_scaling = params['brake_scaling']


        # defined by us
        min_speed = 0.0 #TODO need to check. It seems too strong.
        kp = 0.21
        #kp = 0.8
        ki = 0.00399995
        #ki = 0.0
        kd = 1.0 #3.16046
        #kd = 0.0

        #This equation comes from the slack discussion  (#s_p-system-integrat)
        self.car_constant_for_brake = (self.vehicle_mass + fuel_capacity*GAS_DENSITY)*self.wheel_radius*self.wheel_radius
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.pid_control = PID(kp=kp, ki=ki, kd=kd, mn=self.decel_limit, mx=accel_limit)
        self.lowpass_filter = LowPassFilter(tau=1.0, ts=20.0) #val=current_val*a + prev_val*b,  about a=0.95, b=0.05 (tau=1.0, ts=20.0)
        self.first_control_call = True
    # def control(self, *args, **kwargs):
    def control(self, twist_cmd, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # TODO: We need to modify brake.

        if self.first_control_call is True:
            self.prev_time = rospy.Time.now().to_sec()
            self.first_control_call = False

        if dbw_enabled.data is False:
            self.pid_control.reset()
            return 0,0,0

        filtered_current_velocity = self.lowpass_filter.filt(current_velocity.twist.linear.x)
        # filtered_current_velocity = current_velocity.twist.linear.x
        error = twist_cmd.twist.linear.x - filtered_current_velocity
        current_time = rospy.Time.now().to_sec() #current_velocity.header.stamp.to_sec()
        sample_time = current_time - self.prev_time
        # assert sample_time<=10.0, "{},{},{}".format(sample_time, current_time , self.prev_time)
        # rospy.loginfo("twist_vel={}, cur_time={}, sam_time={}, err={}".format(twist_cmd.twist.linear.x,current_time,sample_time, error))
        if sample_time>1.0:
            rospy.logwarn("sample_time in pid controller is greater than 1.0 seconds! Use 0.01 instead")
            sample_time = 0.01
        elif sample_time <= 0.0:
            sample_time = 0.0001

        self.prev_time = current_time

        throttle = self.pid_control.step(error, sample_time)

        raw_throttle = throttle
        if error >=0 :
            brake = 0
        else:
            acceleration = throttle
            # brake = 40000 #max(self.car_constant_for_brake * throttle,0)
            #

            #desired_decel =  self.decel_limit*0.5  # #TODO We need to change this.
            desired_decel =  self.decel_limit*0.6  # #TODO We need to change this.
            #brake = -desired_decel*self.car_constant_for_brake
            # brake = self.vehicle_mass * self.wheel_radius * desired_decel * (error * 10)
            brake = self.vehicle_mass * self.wheel_radius * desired_decel * (error * 5.4)
            throttle = 0.0

        if throttle > 1.0:
            throttle = 1.0
        elif throttle < 0.0:
            throttle = 0.0


        steer = self.yaw_control.get_steering(linear_velocity=twist_cmd.twist.linear.x, angular_velocity=twist_cmd.twist.angular.z, current_velocity=filtered_current_velocity)
        # throttle = min(throttle * max(1.0, abs((2.0*math.pi- steer)/2.0*math.pi)),0.05)
        throttle = self.throttle_scaling * throttle

        # brake = self.brake_scaling * brake
        # rospy.loginfo("raw_throttle={}, throttle={}, brake={}, steer={}".format(raw_throttle, throttle, brake, steer))
        return throttle, brake, steer
