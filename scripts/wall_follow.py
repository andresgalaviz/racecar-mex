#!/usr/bin/env python

import rospy
import tf
import numpy as np
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan

KP = 0.4
KD = 0.3
SPEED = .8
SPEED_INCREMENT = 2.0

class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan",LaserScan, self.reading, queue_size = 1)
        self.tf = tf.TransformListener()

        #self.sub = rospy.Subscriber("/map",LaserScan, self.reading, queue_size = 1)
        self.Target_distance = 0
        self.Previous_error = 0
        self.Data_size_front_S = 250
        self.Data_size_sides_S = 320
        self.Safe_distance = .7
        self.Wall_distance = 0
        self.One_time = True 

    def control(self, RD, LD, FD):
        '''
        Error = LD - RD
        #Use just one wall if there is just one wall
        if LD > self.Wall_distance: 
            Error = self.Target_distance - RD
        elif RD > self.Wall_distance:
            Error = LD - self.Target_distance
        else:
            if Error > 0:
                self.Target_distance = LD - Error
            else:
                self.Target_distance = LD + Error
        '''
        Error = LD - self.Target_distance
        Slope = self.Previous_error - Error
        Target_angle = Error * KP + Slope * KD

        #print Target_angle
        # Avoid wall crashing working on it
        if FD < (self.Wall_distance/2):
            if Target_angle > 0:
                Target_angle += .5
            else:
                Target_angle -= .5

        self.Previous_error = Error
        #print self.tf.allFramesAsString()
        #self.tf.waitForTransform("chassis", "left_front_wheel", rospy.Time(0), rospy.Duration(.1))

        self.Transform = self.tf.lookupTransform("base_link", "map", rospy.Time(0))[0]
        print self.Target_distance
        print self.Transform , self.Transform_Start
        #print Target_angle
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        if -0.04 < Target_angle < 0.04 and FD > self.Wall_distance:
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
        Data_size_front = 100
        Data_size_left = 320
        Data_size_right = 320
        Angle_List = []
        Ranges_List = []

        #Create list of angles and ranges
        for i in range(len(msg.ranges)):
            Angle_List.append(a_inc * i + min_a) 
            Ranges_List.append(msg.ranges[i])
        
        #Filtering and dereminating the laser scaner
        for i in range(self.Data_size_sides_S):
            if i == 0:
                RDI = int(((-np.pi/2 + max_a)/a_inc) - 70)
                LDI = int(((np.pi/2 - max_a)/a_inc) - 70)
                FDI = int((len(msg.ranges)/2) - Data_size_front/2)
                Right_distance = Ranges_List[RDI] * -np.sin(Angle_List[RDI])
                Left_distance = Ranges_List[LDI] * np.sin(Angle_List[LDI])
                Front_distance = Ranges_List[FDI]
                #print Left_distance , Right_distance, Front_distance

            else:
                RDI += 1
                LDI -= 1
                FDI += 1
                RD = Ranges_List[RDI] * -np.sin(Angle_List[RDI])
                LD = Ranges_List[LDI] * np.sin(Angle_List[LDI])
                FD = Ranges_List[FDI]
                #print RDI , LDI, FDI
                #Filter outer values
                if i < Data_size_right:
                    if RD > (Right_distance/i) + self.Safe_distance:
                        Right_distance += Right_distance/i
                    else:
                        Right_distance += RD
                if i < Data_size_left:
                    if LD > (Left_distance/i) + self.Safe_distance:
                        Left_distance += Left_distance/i
                    else:
                        Left_distance += LD

                if i < Data_size_front:
                    if FD > (Front_distance/i) + self.Safe_distance:
                        Front_distance += Front_distance/i
                    else:
                        Front_distance += FD

        Right_distance = Right_distance/Data_size_right
        Left_distance = Left_distance/Data_size_left
        Front_distance = Front_distance/Data_size_front
        
        if self.One_time:
            self.Wall_distance = Left_distance + Right_distance
            self.Target_distance = (self.Wall_distance)/2
            self.tf.waitForTransform("base_link", "map", rospy.Time(0), rospy.Duration(.5))
            self.Transform_Start = self.tf.lookupTransform("base_link", "map", rospy.Time(0))[0]
            self.One_time = False

        #print Left_distance , Right_distance, Front_distance
        self.control(Right_distance, Left_distance, Front_distance)
        

if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow()
    rospy.spin()