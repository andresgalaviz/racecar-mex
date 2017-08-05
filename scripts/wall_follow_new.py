#!/usr/bin/env python
import rospy
import tf
import math
import time
import cv2
import numpy as np
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge


#Center
KP = 1.5
KD = .06
KI = .02
'''
KP = 1.5
KD = 0.06
KI = .02
'''
SPEED = 4
SPEED_INCREMENT = 1.0
SPEED_DEC = 2.6
SPEED_MAX = SPEED
class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan",LaserScan, self.reading, queue_size = 1)
        self.sub_zed = rospy.Subscriber("/zed/rgb/image_rect_color",Image, self.reading_zed, queue_size = 1)
        self.debugImagePub = rospy.Publisher(visualDebugTopic, Image, queue_size = 1)
        self.Target_distance = 0
        self.Previous_error = 0
        self.Data_size_front_S = 250
        self.Data_size_sides_S = 270
        self.Safe_distance = .35
        self.Wall_distance = 0
        self.One_time = True
        self.Error_Integral = 0
        self.Direction = "Vision"
        self.Vision_error = 0
        self.bridge = CvBridge()
    def control(self, RD, LD, FD):
        #Use just one wall if there is just one wall
        if self.Direction == "Right": 
            #Error = self.Test_error
            Error = self.Target_distance - RD
            self.move = True
        elif self.Direction == "Left":
            Error = LD - self.Target_distance
            self.move = True
        elif self.Direction == "Center":
            Error = self.Test_error
            self.move = True
        elif self.Direction == "Vision":
            Error = self.Vision_error 
            self.move = True
        else:
            self.move = False
        if self.move == False:
            Error = 0
        Slope = (Error - self.Previous_error) / (self.currtm - self.prevtm)
        self.Error_Integral = (Error + self.Previous_error) / (self.currtm - self.prevtm)

        if self.Error_Integral > .1:
            self.Error_Integral = 0

        Target_angle = Error * KP + Slope * KD + self.Error_Integral * KI
        
        # Avoid wall crashing working on it
        saturation_value = .6

        if FD < (.4):
            if Target_angle > 0:
                
                SPEED = SPEED_DEC
                Target_angle += .05
            else:
                SPEED = SPEED_DEC
                Target_angle -= .05
        else:
            SPEED = SPEED_MAX
        
        if Target_angle > saturation_value:
            Target_angle = saturation_value

        elif Target_angle < -saturation_value:
            Target_angle = -saturation_value
        print "Target angle = ", Target_angle
        #print "Target angle = ", Target_angle, " LD = ", LD, " RD = ", RD, " Target Distance = ", self.Target_distance 
        self.Previous_error = Error
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()

        if -0.07 < Target_angle < 0.07 and FD > self.Wall_distance and self.move:
            global SPEED
            drive_msg.speed = SPEED * SPEED_INCREMENT
        else :
            drive_msg.speed = SPEED

        drive_msg.steering_angle = Target_angle
        drive_msg_stamped.drive = drive_msg

        #print Target_angle, LD, RD, self.Target_distance
        if self.move:
            self.pub.publish(drive_msg_stamped)
        else:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0
            stop_msg.header.stamp = rospy.Time.now()
            self.pub.publish(stop_msg)

    def reading_zed(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        errorMsg, debugImage = self.visualProcessor.computeError(image)
        self.errorPub.publish(errorMsg)
        debugImageMsg = self.bridge.imgmsg_to_cv2(debugImage, 'bgr8')
        self.debugImagePub.publish(debugImageMsg)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Left
        cv.CvtColor(imageL,hsv,cv2.COLOR_BGR2HSV)
        # Right
        #hsv_imageR=cv.CreateImage(cv.GetSize(imageR),8,1)
        #cv.CvtColor(imageR,hsv_imageR,cv.COLOR_BGR2HSV)
        ymn = np.array([0,160,180])
        ymx = np.array([255,255,255])
        mask = cv2.inRange(hsv, ymn, ymx)
        res = cv2.bitwise_and(frame, frame, mask = mask)
        #cv2.imshow('mask', mask)

        width = len(mask[0])
        height = len(mask)
        print "W =", width, " H = " , height
        threshhold = []
        for i in range(height/4 ,height*3/4):
            for j in range(width):
                if mask[i][j] == 255:
                    threshhold.append(j)
        Idex_average = np.average(np.array(threshhold))

        #print " Index Average = ", Idex_average - 540
        self.Vision_error = (Idex_average - (width/2))/(width/2)
        
        cv2.waitKey(5)

    def reading(self, msg):
        #print msg.ranges
        max_a = msg.angle_max
        min_a = msg.angle_min
        a_inc = msg.angle_increment
        Data_size_front = 100
        Data_size_left = 270
        Data_size_right = 270
        Angle_List = []
        Ranges_List = []

        #Create list of angles and ranges
        for i in range(70, len(msg.ranges)-70):
            if (i >= 535 or i <= 545) and msg.range[i] < self.Safe_distanc :
                self.move = False
                break
            Angle_List.append(a_inc * i + min_a) 
            Ranges_List.append(msg.ranges[i])
        #average
        Average = np.average(np.array(Ranges_List))
        Stand_deviation = np.std(np.array(Ranges_List))
        #higher value location
        threshold = []

        for i in range(len(Ranges_List)):
            if Ranges_List[i] > Average:
                threshold.append(i)
        #decition making
        '''
       spikes = [1]
        spikes_idex = [0]
        x = 0
        for i in range(len(threshold) - 1):
            if threshold[i] + 1 == threshold[i+1]:
                spikes[x] =+ 1
            else:
                if i < len(threshold):
                    spikes.append(1)
                    spikes_idex.append(i+1)
                x =+ 1
        mayor = 0
        for i in range(len(spikes)):
            if spikes[mayor] < spikes[i]:
                mayor = i
        Idex_average = 0
        
        for i in range(spikes_idex[mayor-1], spikes_idex[mayor]):
            Idex_average =+ threshold[i] 
        Idex_average = Idex_average/spikes[mayor]
        '''
        Idex_average = np.average(np.array(threshold))
        
        print " Index Average = ", Idex_average - 470
        self.Test_error = (Idex_average - 470)/470

        #Filtering and dereminating the laser scaner
        for i in range(self.Data_size_sides_S):
            if i == 0:
                RDI = int(((-np.pi/2 + max_a)/a_inc) - 70)
                #LDI = -int(((np.pi/2 + max_a)/a_inc) - 70)
                LDI = 1080-Data_size_left
                FDI = int((len(Ranges_List)/2) - Data_size_front/2)
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
            self.Target_distance = .6
            # self.Target_distance = (self.Wall_distance)/2 - (self.Wall_distance *.1)
            #self.tf.waitForTransform("base_link", "map", rospy.Time(0), rospy.Duration(.5))
            #self.Transform_Start = self.tf.lookupTransform("base_link", "map", rospy.Time(0))[0]
            #self.Direction = "Right"
            self.Flag = 0
            self.One_time = False
            self.currtm = time.time()
            self.prevtm = self.currtm

        self.currtm = time.time()
        self.control(Right_distance, Left_distance, Front_distance)
        self.prevtm = self.currtm


if __name__=="__main__":
    rospy.init_node("wall_follow")
    #cap = cv2.VideoCapture(0)
    WallFollow()
    rospy.spin()
