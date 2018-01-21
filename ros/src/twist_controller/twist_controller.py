from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    # def __init__(self, *args, **kwargs):
    def __init__(self, params):
        # TODO: Implement

        vehicle_mass = params['vehicle_mass']
        fuel_capacity = params['fuel_capacity']
        self.brake_deadband = params['brake_deadband']
        decel_limit = params['decel_limit']
        accel_limit = params['accel_limit']
        wheel_radius = params['wheel_radius']
        wheel_base = params['wheel_base']
        steer_ratio = params['steer_ratio']
        max_lat_accel = params['max_lat_accel']
        max_steer_angle = params['max_steer_angle']

        # defined by us
        min_speed = 0.0 #TODO need to check
        kp = 0.21
        ki = 0.00399995
        kd = 3.16046

        #This equation comes from the slack discussion  (#s_p-system-integrat)
        self.car_constant_for_brake = (vehicle_mass + fuel_capacity*GAS_DENSITY)*wheel_radius
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.pid_control = PID(kp=kp, ki=ki, kd=kd, mn=decel_limit, mx=accel_limit)
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

        if dbw_enabled is False:
            self.pid_control.reset()
        filtered_current_velocity = self.lowpass_filter.filt(current_velocity.twist.linear.x)
        # filtered_current_velocity = current_velocity.twist.linear.x
        error = twist_cmd.twist.linear.x - filtered_current_velocity
        current_time = rospy.Time.now().to_sec() #current_velocity.header.stamp.to_sec()
        sample_time = current_time - self.prev_time
        # assert sample_time<=10.0, "{},{},{}".format(sample_time, current_time , self.prev_time)
        # rospy.loginfo("twist_vel={}, cur_time={}, sam_time={}, err={}".format(twist_cmd.twist.linear.x,current_time,sample_time, error))
        if sample_time>1.0:
            rospy.logerr("sample_time in pid controller is greater than 1.0 seconds! Use 0.01 instead")
            sample_time = 0.01
        elif sample_time <= 0.0:
            sample_time = 0.0001

        self.prev_time = current_time

        throttle = self.pid_control.step(error, sample_time)

        if error >=0 :
            brake = 0
        else:
            acceleration = throttle
            brake = max(self.car_constant_for_brake * throttle,0) #TODO: I used throttle but it should be acceleration
            throttle = 0.0

        if throttle > 1.0:
            throttle = 1.0
        elif throttle < 0.0:
            throttle = 0.0


        steer = self.yaw_control.get_steering(linear_velocity=twist_cmd.twist.linear.x, angular_velocity=twist_cmd.twist.angular.z, current_velocity=filtered_current_velocity)

        #rospy.loginfo("throttle={}, brake={}, steer={}".format(throttle, brake, steer))
        return throttle, brake, steer
