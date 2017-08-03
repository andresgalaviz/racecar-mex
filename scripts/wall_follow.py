#!/usr/bin/env python
import rospy
import tf
import math
import time
import numpy as np
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan

KP = 1.5
KD = .7
KI = .1
SPEED = 1
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
        self.Error_Integral = 0

    def control(self, RD, LD, FD):
        #Use just one wall if there is just one wall
        '''if self.Direction == "Right": 
            Error = self.Test_error
            #Error = self.Target_distance - RD
            move = True
        elif self.Direction == "Left":
            Error = LD - self.Target_distance
            move = True
        elif self.Direction == "Center":
            Error = LD - RD
            move = True
        else:
            move = False
        '''
        move = True
        Error = self.Test_error
        Slope = (Error - self.Previous_error) / (self.currtm - self.prevtm)
        self.Error_Integral = (Error + self.Previous_error) / (self.currtm - self.prevtm)

        if self.Error_Integral > .5:
            self.Error_Integral = 0

        Target_angle = Error * KP + Slope * KD + self.Error_Integral * KI
        
        # Avoid wall crashing working on it
        saturation_value = .6

        if FD < (self.Wall_distance/2):
            if Target_angle > 0:
                Target_angle += .05
            else:
                Target_angle -= .05
        
        if Target_angle > saturation_value:
            Target_angle = saturation_value

        elif Target_angle < -saturation_value:
            Target_angle = -saturation_value

        print "Target angle = ", Target_angle
        self.Previous_error = Error
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()

        if -0.04 < Target_angle < 0.04 and FD > self.Wall_distance and move:
            global SPEED
            drive_msg.speed = SPEED * SPEED_INCREMENT
        else :
            drive_msg.speed = SPEED

        drive_msg.steering_angle = Target_angle
        drive_msg_stamped.drive = drive_msg

        #print Target_angle, LD, RD, self.Target_distance
        if move:
            self.pub.publish(drive_msg_stamped)
        else:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0
            stop_msg.header.stamp = rospy.Time.now()
            self.pub.publish(stop_msg)

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
        #average
        Average = np.average(np.array(msg.ranges))
        Stand_deviation = np.std(np.array(msg.ranges))
        #higher value location
        threshold = []
        for i in range(len(msg.ranges)):
            if Ranges_List[i] > Average:
                threshold.append(i)
        Idex_average = np.average(np.array(threshold))
        #print " Index Average = ", Idex_average - 540
        self.Test_error = (Idex_average - 540)/540

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
            self.Target_distance = (self.Wall_distance)/2 - (self.Wall_distance *.1)
            self.tf.waitForTransform("base_link", "map", rospy.Time(0), rospy.Duration(.5))
            self.Transform_Start = self.tf.lookupTransform("base_link", "map", rospy.Time(0))[0]
            self.Direction = "Center"
            self.Flag = 0
            self.One_time = False
            self.currtm = time.time()
            self.prevtm = self.currtm
          
        self.Transform = self.tf.lookupTransform("base_link", "map", rospy.Time(0))[0]
        self.Dist_Start = math.floor(math.sqrt((self.Transform[0])**2 + (self.Transform[1])**2))
        if self.Flag == 0 and self.Dist_Start > 1:
            self.Flag += 1
        elif self.Flag == 1 and self.Dist_Start == 0:
            self.Direction = "Right"
            self.Error_Integral = 0
            self.Flag += 1
        elif self.Flag == 2 and self.Dist_Start > 1:
            self.Flag += 1
        elif self.Flag == 3 and self.Dist_Start == 0:
            self.Direction = ""
        #print Left_distance , Right_distance, Front_distance
        self.currtm = time.time()
        self.control(Right_distance, Left_distance, Front_distance)
        self.prevtm = self.currtm

if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow()
    rospy.spin()