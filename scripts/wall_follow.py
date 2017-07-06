#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan

KP = 0.4
KD = 0.3
#TARGET_DISTANCE = 3.0
SPEED = 1
SPEED_INCREMENT = 2.0
PREVIOUS_ERROR = 0

class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan",LaserScan, self.reading, queue_size = 1)

    def control(self, RD, LD, FD):
        #Error = TARGET_DISTANCE - RD
        Error = LD - RD
        global PREVIOUS_ERROR 
        Slope = PREVIOUS_ERROR - Error
        Target_angle = Error * KP + Slope * KD
        #print Target_angle
        #Target_angle = Error * KP
        # Avoid wall crashing working on it
        if FD < 2.5:
            if Target_angle > 0:
                Target_angle += .5
            else:
                Target_angle -= .5

        PREVIOUS_ERROR = Error

        #print Target_angle
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        if -0.04 < Target_angle < 0.04 and FD > 4.5:
            global SPEED
            drive_msg.speed = SPEED * SPEED_INCREMENT
        else:
            drive_msg.speed = SPEED
        drive_msg.steering_angle = Target_angle
        drive_msg_stamped.drive = drive_msg
        self.pub.publish(drive_msg_stamped)
        
    def reading(self, msg):
        #print msg.ranges
        max_a = msg.angle_max
        min_a = msg.angle_min
        a_inc = msg.angle_increment
        Safe_distance = .7
        Data_size_front = 250
        Data_size_sides = 320

        # Trying to get an average of values insead of just 3 laser scans 
        #print a_inc
        for i in range(Data_size_sides):
            if i == 0:
                RD = int(((-np.pi/2 + max_a)/a_inc) - 70)
                LD = int(((np.pi/2 - max_a)/a_inc) - 70)
                FD = int(max_a/a_inc) - (Data_size_sides/2)
                Right_distance = msg.ranges[RD]
                Left_distance = msg.ranges[LD]
                Front_distance = msg.ranges[FD]

            else:
                RD += 1
                LD -= 1
                FD += 1
                #Filter outer values
                if msg.ranges[RD] > (Right_distance/i) + Safe_distance :
                    Right_distance += Right_distance/i
                else:
                    Right_distance += msg.ranges[RD]

                if msg.ranges[LD] > (Left_distance/i) + Safe_distance:
                    Left_distance += Left_distance/i
                else:
                    Left_distance += msg.ranges[LD]

                if i < Data_size_front:
                    if msg.ranges[FD] > (Front_distance/i) + Safe_distance:
                        Front_distance += Front_distance/i
                    else:
                        Front_distance += msg.ranges[FD]

        Right_distance = Right_distance/Data_size_sides
        Left_distance = Left_distance/Data_size_sides
        Front_distance = Front_distance/Data_size_front


        self.control(Right_distance, Left_distance, Front_distance)
        #print Left_distance , Right_distance, Front_distance



if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow()
    rospy.spin()