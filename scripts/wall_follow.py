#!/usr/bin/env python
import rospy

import sys, select, termios, tty

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

SHOW_VIS = False
FAN_ANGLE = np.pi/5.0
TARGET_DISTANCE = 1.0
MEDIAN_FILTER_SIZE=141
KP = 0.4 # distance term
KD = 0.3  # angle term
# KD = 0.5  # angle term
PUBLISH_LINE = True
HISTORY_SIZE = 5 # Size of the circular array for smoothing steering commands
PUBLISH_RATE = 20.0 # number of control commands to publish per second
SPEED = 1.0

EPSILON = 0.000001

banner = """
Reading from the keyboard  and Publishing to AckermannDriveStamped!
---------------------------
Moving around:
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan", LaserScan, self.lidarCB, queue_size=1)

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
        
    def fit_line(self):
        if self.received_data and self.xs.shape[0] > 0:
            # fit line to euclidean space laser data in the window of interest
            slope, intercept, r_val, p_val, std_err = stats.linregress(self.xs,self.ys)
            self.m = slope
            self.c = intercept

    def lidarCB(self, msg):
        if not self.received_data:
            #rospy.loginfo("success! first message received")

            # populate cached constants
            center_angle = 0

            self.min_angle = center_angle - FAN_ANGLE
            self.max_angle = center_angle + FAN_ANGLE
            self.laser_angles = (np.arange(len(msg.ranges)) * msg.angle_increment) + msg.angle_min

        #-2.09439992905 2.09439992905 0.00387851847336 0.10000000149 10.0 1081
        #msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max, len(msg.ranges)

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

        # convert from polar to euclidean coordinate space
        self.ys = filtered_ranges * np.cos(filtered_angles)
        self.xs = filtered_ranges * np.sin(filtered_angles)

        self.fit_line()
        self.compute_pd_control()

        # filter lidar data to clean it up and remove outlisers
        self.received_data = True

    def compute_pd_control(self):
	    print "computing control"

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.5
turn = 0.25

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    WallFollow()
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size = 0)
    rospy.init_node('keyop')

    x = 0
    th = 0
    status = 0

    try:
        while 1:
            key = getKey()
            if key in keyBindings.keys():
                x = keyBindings[key][0]
                th = keyBindings[key][1]
            else:
                x = 0
                th = 0
                if key == '\x03':
                    break
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"

            msg.drive.speed = x*speed
            msg.drive.acceleration = 1
            msg.drive.jerk = 1
            msg.drive.steering_angle = th*turn
            msg.drive.steering_angle_velocity = 1

            pub.publish(msg)

    except:
        print 'error'

    finally:
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        msg.drive.speed = 0
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = 0
        msg.drive.steering_angle_velocity = 1
        pub.publish(msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
