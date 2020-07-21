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


PUBLISH_LINE = True

SHOW_VIS = False
FAN_ANGLE = np.pi/4.0
TARGET_DISTANCE = 3.0
MEDIAN_FILTER_SIZE = 141
KP = 0.4 # distance term
KD = 0.3  # angle term
SPEED = 5.0
# KD = 0.5  # angle term
HISTORY_SIZE = 5
PUBLISH_RATE = 20
SPEED = 1.0

EPSILON = 0.000001

class CircularArray(object):
    """docstring for CircularArray"""
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])

class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop",\
                AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan", LaserScan, self.lidarCB, queue_size=1)

        self.center_angles = [(-math.pi /2), 0, (math.pi/2)]
        self.direction_multiplier = [1,0,-1]

        self.line_pub = [None]*len(self.center_angles)

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
        self.laser_angles = None

        self.drive_thread = Thread(target=self.apply_control)
        self.drive_thread.start()

        if PUBLISH_LINE:
            for i in range(len(self.center_angles)):
                self.line_pub[i] = rospy.Publisher("/viz/line_fit_%d"%i, PolygonStamped, queue_size =1)

        self.steering_hist = CircularArray(HISTORY_SIZE)

    def publish_line(self, i):
        # find the two points that intersect between the fan angle lines and the found y=mx+c line
        x0 = self.c / (np.tan(FAN_ANGLE) - self.m)
        x1 = self.c / (-np.tan(FAN_ANGLE) - self.m)

        y0 = self.m*x0+self.c
        y1 = self.m*x1+self.c

        poly = Polygon()
        p0 = Point32()
        p0.y = x0
        p0.x = y0

        p1 = Point32()
        p1.y = x1
        p1.x = y1
        poly.points.append(p0)
        poly.points.append(p1)

        polyStamped = PolygonStamped()
        polyStamped.header.frame_id = "base_link"
        polyStamped.polygon = poly

        self.line_pub[i].publish(polyStamped)

    # given line parameters cached in the self object, compute the pid control
    def drive_straight(self):
        while not rospy.is_shutdown():
            drive_msg_stamped = AckermannDriveStamped()
            drive_msg = AckermannDrive()
            drive_msg.speed = 2.0
            drive_msg.steering_angle = 0.0
            drive_msg.acceleration = 0
            drive_msg.jerk = 0
            drive_msg.steering_angle_velocity = 0
            drive_msg_stamped.drive = drive_msg
            self.pub.publish(drive_msg_stamped)

            # don't spin too fast
            rospy.sleep(1.0/PUBLISH_RATE)

    def apply_control(self):
        while not rospy.is_shutdown():
            if self.control is None:
                print "No control data"
                rospy.sleep(0.5)
            else:
                self.steering_hist.append(self.control[0])
                # smoothed_steering = self.steering_hist.mean()
                smoothed_steering = self.steering_hist.median()

                # print smoothed_steering, self.control[0]
                drive_msg_stamped = AckermannDriveStamped()
                drive_msg = AckermannDrive()
                drive_msg.speed = self.control[1]
                drive_msg.steering_angle = smoothed_steering
                drive_msg.acceleration = 0
                drive_msg.jerk = 0
                drive_msg.steering_angle_velocity = 0
                drive_msg_stamped.drive = drive_msg
                self.pub.publish(drive_msg_stamped)

                rospy.sleep(1.0/PUBLISH_RATE)

    # given line parameters cached in the self object, compute the pid control
    def compute_pd_control(self):
        if self.received_data:
            # given the computed wall slope, compute theta, avoid divide by zero error
            if np.abs(self.m) < EPSILON:
                theta = 0
                x_intercept = 0
            else:
                theta = np.arctan(self.m)
                # solve for y=0 in y=mx+c
                x_intercept = self.c / self.m

            # x axis is perp. to robot but not perpindicular to wall
            # sine term solves for minimum distance to wall
            wall_dist = np.abs(np.sin(theta)*x_intercept)
            #print self.min_angle, ": ", wall_dist

            # control proportional to angular error and distance from wall
            distance_term = self.center_angles[2] * KP * (wall_dist - TARGET_DISTANCE)
            angle_term = KD * theta
            control = angle_term + distance_term
            # avoid turning too sharply
            self.control = (np.clip(control, -0.3, 0.3), SPEED)

    def fit_line(self):
        if self.received_data and self.xs.shape[0] > 0:
            # fit line to euclidean space laser data in the window of interest
            slope, intercept, r_val, p_val, std_err = stats.linregress(self.xs, self.ys)
            self.m = slope
            self.c = intercept
        # print "SLOPE: %.4f"%(self.m)
        # print "INTERCEPT: %.4f"%(self.c)

    # window the data, compute the line fit and associated control
    def lidarCB(self, msg):
        if not self.received_data:
            rospy.loginfo("success! first message received")
            self.laser_angles = (np.arange(len(msg.ranges)) * msg.angle_increment) + msg.angle_min

        for i, center_angle in enumerate(self.center_angles):
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

            # convert from polar to euclidean coordinate space
            self.ys = filtered_ranges * np.cos(filtered_angles)
            self.xs = filtered_ranges * np.sin(filtered_angles)

            # for i in range(len(self.ys)):
            #     print "%.4f, %.4f"%(self.xs[i], self.ys[i])

            self.fit_line()
            self.compute_pd_control()
            if PUBLISH_LINE:
                self.publish_line(i)
            
        # filter lidar data to clean it up and remove outlisers
        self.received_data = True


if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow()
    rospy.spin()