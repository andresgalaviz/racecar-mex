#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Header
import numpy as np
from threading import Thread #imsosorry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from scipy import signal, stats
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import Polygon, Point32, PolygonStamped

RIGHT = 'right'
LEFT  = 'left'
BOTH = 'both'


SHOW_VIS = False
FAN_ANGLE = np.pi/5.0
TARGET_DISTANCE = 1.0
MEDIAN_FILTER_SIZE=141
KP = 0.4 # distance term
KD = 0.3  # angle term
# KD = 0.5  # angle term

EPSILON = 0.000001

class WallFollow():
    def __init__(self, direction):
        if direction not in [RIGHT, LEFT]:
            rospy.loginfo("incorect %s wall selected.  choose left or right")
            rospy.signal_shutdown()
        self.direction = direction
        
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop",\
                AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan", LaserScan, self.lidarCB, queue_size=1)
        
        # computed control instructions
        self.control = None

        # containers for laser scanner related data
        self.data = None
        self.xs = None
        self.ys = None
        self.m = 0
        self.c = 0
        
        # flag to indicate the first laser scan has been received
        self.received_data = False
        
        # cached constants
        self.min_angle = None
        self.max_angle = None
        self.direction_muliplier = 0
        self.laser_angles = None

        self.center_angles = [(-math.pi /2), 0, (math.pi/2)]

    # given line parameters cached in the self object, compute the pid control
    def compute_pd_control(self):
        # print "compute pd control"
        print ""

    def fit_line(self):
        if self.received_data and self.xs.shape[0] > 0:
            # fit line to euclidean space laser data in the window of interest
            slope, intercept, r_val, p_val, std_err = stats.linregress(self.xs,self.ys)
            self.m = slope
            self.c = intercept
        # print "SLOPE: %.4f"%(self.m)
        # print "INTERCEPT: %.4f"%(self.c)
            
    # window the data, compute the line fit and associated control
    def lidarCB(self, msg):
        if not self.received_data:
            rospy.loginfo("success! first message received")
            self.laser_angles = (np.arange(len(msg.ranges)) * msg.angle_increment) + msg.angle_min

        for center_angle in self.center_angles:
            self.min_angle = center_angle - FAN_ANGLE
            self.max_angle = center_angle + FAN_ANGLE
                
            self.data = msg.ranges
            values = np.array(msg.ranges)

            # remove out of range values
            ranges = values[(values > msg.range_min) & (values < msg.range_max)]
            angles = self.laser_angles[(values > msg.range_min) & (values < msg.range_max)]

            # apply median filter to clean outliers
            filtered_ranges = signal.medfilt(ranges, MEDIAN_FILTER_SIZE)

            # apply a window function to isolate values to the side of the car
            window = (angles > self.min_angle) & (angles < self.max_angle)
            filtered_ranges = filtered_ranges[window]
            filtered_angles = angles[window]
            print "%.4f : "% center_angle, filtered_ranges[0], filtered_ranges[len(filtered_ranges)-1]

            # convert from polar to euclidean coordinate space
            self.ys = filtered_ranges * np.cos(filtered_angles)
            self.xs = -1*filtered_ranges * np.sin(filtered_angles)

            # for i in range(len(self.ys)):
            #     print "%.4f, %.4f"%(self.xs[i], self.ys[i])


            self.fit_line()
            self.compute_pd_control()
        print ""
        # filter lidar data to clean it up and remove outlisers
        self.received_data = True

if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow(RIGHT)
    rospy.spin()