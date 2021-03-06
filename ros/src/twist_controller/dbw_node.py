#!/usr/bin/env python

import math
import numpy
import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import TwistController
from yaw_controller import YawController

mps_2_MPH = 1.0 / 0.44704
MPH_2_mps = 0.44704

min_speed = 1.0    #TODO: May need to adjust this variable

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        
        self.current_linear_velocity = 0.0
        self.target_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.dbw_enabled = False

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8) 
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)      # meters/s^2
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)  #=458 degrees

        # steer:wheel = 14.8:1,  
        # max_steer_angle = 8 rads  = 458 degrees
        # 458/14.8 = ~30 degrees (max steering range: -15 to 15)
        # steering_sensitivity is in radians^(-1)
        # this number is used to get the noramlized steering 
        self.steering_sensitivity = steer_ratio / max_steer_angle * 2.0 

        #rospy.logerr('st_sen:{} {} {}'.format( steer_ratio, max_steer_angle, self.steering_sensitivity))

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        self.controller = TwistController() #(<Arguments you wish to provide>)
        #max_steer_angle is half as in yaw_controller, we have range of (-1* max_steer_angle to max_steer_angle)

        self.controller.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity',TwistStamped, self.velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_cb)

        rospy.spin() # ===> use spin, publishing cmd right away in twist_cb

    def dbw_cb(self, msg):
        if (self.dbw_enabled != msg.data):
            self.dbw_enabled = msg.data
        pass

    def velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.z
        pass

    def twist_cb(self, msg):
        self.target_linear_velocity = msg.twist.linear.x
        self.target_angular_velocity = msg.twist.angular.z	

        target_linear_velocity = self.target_linear_velocity * MPH_2_mps

        if self.dbw_enabled:  #<dbw is enabled>:
            throttle, brake, steer = self.controller.control(self.current_linear_velocity, target_linear_velocity, self.steering_sensitivity, self.target_angular_velocity)

            self.publish(throttle, brake, steer)

            #if brake > 10:
            #    rospy.logerr('dbw_node:{: f}\t{: f}\t{: f}'.format(steer, throttle, brake))
    
        pass


    def publish(self, throttle, brake, steer):

        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        ''' brake command will set throttle value to 0, even if brake is zero'''
        ''' therefore the brake command is only sent if we are braking '''

        if brake > 0:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
