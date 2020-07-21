#!/usr/bin/env python
import math
import time
import numpy as np

import rospy
from std_msgs.msg import String, Header, Float32
from ackermann_msgs.msg import AckermannDriveStamped

from lab4.msg import EulerAngle

"""
Author: Jesse Chang
This program tells the robot to drive according to a specific curvature
"""

class PID():
    def init(self):
        #initialize
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.Initialize()

    def Initialize(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm
        

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0


    def controller_update(self, error):
        """ Performs a PID computation and returns a control value."""

        self.currtm = time.time()
        dt = self.currtm - self.prevtm          # get delta t
        de = error - self.prev_err              # get delta error

        self.Cp = self.Kp * error               # proportional term
        self.Ci += error * dt                   # integral term

        self.Cd = 0
        if dt > 0:
            self.Cd = de/dt                     # derivative term

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error

        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)

class LineFollower():
    def init(self):
        '''
        Init the node. See safety.py for a good idea of what to do.
        '''
        self.steeringPID = PID() # create controller for setting the wheels

        # Init subscribers and publishers
        self.sub_euler_angle = rospy.Subscriber("/euler_angle", EulerAngle, self.euler_angle_callback, queue_size=1)
        self.sub_curvature = rospy.Subscriber("/curve_detect/curvature", Float32, self.error_callback, queue_size = 1)

        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )

        #set desired max speed, distance
        self.proj_curvature = 0
        self.current_curvature = 0
        self.phi = 0
        self.current_radius = 0

        #set control gains
        self.steeringPID.Kp = 0.45
        self.steeringPID.Ki = 0
        self.steeringPID.Kd = 0

        self.steer_angle = 0
        self.velocity = 0.5

        self.i = 0
        rospy.loginfo("Controller initialized")

    def publish_commands(self):
        curvature_error = math.atan(self.proj_curvature) - math.atan(self.current_curvature)
        control_value = self.steeringPID.controller_update(curvature_error)
        self.i += 1
        if self.i % 10 == 0:
            print '%6f\t%6f\t%6f' % (self.current_curvature - self.proj_curvature, self.proj_curvature, self.current_curvature)
        self.steer_angle = self.cutoff_steering_angle(self.steer_angle + control_value)
        #publish control commands
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = self.steer_angle
        drive_msg.drive.speed = self.velocity
        drive_msg.drive.steering_angle_velocity = 0.1 * math.pi/2 #rad/s CONSTANT
        drive_msg.drive.acceleration = 0.0
        drive_msg.drive.jerk = 0.0

        self.pub.publish(drive_msg)

    def cutoff_steering_angle(self, angle):

        '''
        makes the steering angle between -0.27 and 0.27
        takes in a float, angle
        returns a float, angle_corrected
        can use this function to normalize steering angle later
        '''
        
        #scale_factor = 0.8
        #angle = angle/scale_factor
        #rospy.loginfo("raw angle " +  str(angle))
        if angle < -0.27:
            result = -0.27
        elif angle > 0.27:
            result = 0.27
        else:
            result = angle

        return result

    def euler_angle_callback(self, msg):
        self.current_phi = msg.phi
        self.current_curvature = msg.p_curve
        self.current_radius = msg.p_rad
        self.publish_commands()

    def error_callback(self, msg):
        self.proj_curvature = msg.data
        #self.publish_commands()

    def curvature_to_steering(self, k):
        return 0.287 * k


if name=="main":
    # Tell ROS that we're making a new node.
    rospy.init_node("Line_Follower_Node")

    # Init the node
    line_follower = LineFollower()

    rospy.spin()
    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    line_follower.publish_commands()
#    r.sleep()