#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan

KP = 0.4
KD = 0.3
#TARGET_DISTANCE = 3.0
SPEED = .8
SPEED_INCREMENT = 2

class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size =1 )
        rospy.Subscriber("/scan",LaserScan, self.reading, queue_size = 1)

    def control(self, RD, LD, FD):
        #pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size =1 )
        #TARGET_DISTANCE = (RD + LD) / 2
        #Error = TARGET_DISTANCE - RD
        Error = LD - RD
        Target_angle = Error * KP
        # Avoid wall crashing working on it
        if FD < 2.5:
            if Target_angle > 0:
                Target_angle += .5
            else:
                Target_angle -= .5

        #print Target_angle
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        if -0.04 < Target_angle < 0.04 and FD > 4.5:
            drive_msg.speed = SPEED * SPEED_INCREMENT
        else:
            drive_msg.speed = SPEED
        
        drive_msg.speed = SPEED
        drive_msg.steering_angle = Target_angle
        drive_msg_stamped.drive = drive_msg
        self.pub.publish(drive_msg_stamped)
        
    def reading(self, msg):
        #print msg.ranges
        max_a = msg.angle_max
        min_a = msg.angle_min
        a_inc = msg.angle_increment
        Data_size = 100
        # Trying to get an average of values insead of just 3 laser scans 
        for i in range(Data_size):
            if i == 0:
                RD = int((-np.pi/2 + max_a)/a_inc)
                LD = int((np.pi/2 - max_a)/a_inc)
                FD = int(max_a/a_inc) - (Data_size/2)
                Right_distance = msg.ranges[RD]
                Left_distance = msg.ranges[LD]
                Front_distance = msg.ranges[FD]
            else:
                RD += 1
                LD += 1
                FD +=   1
                Right_distance += msg.ranges[RD]
                Left_distance += msg.ranges[LD]
                Front_distance += msg.ranges[FD]

        Right_distance = Right_distance/Data_size
        Left_distance = Left_distance/Data_size
        Front_distance = Front_distance/Data_size
        self.control(Right_distance, Left_distance, Front_distance)
        #print Left_distance , Right_distance, Front_distance



if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow()
    rospy.spin()