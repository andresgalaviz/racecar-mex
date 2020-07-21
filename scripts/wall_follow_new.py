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
from ar_track_alvar_msgs.msg import AlvarMarkers
from timeit import default_timer as timer

#Center
KP = 1.2
KD = .4
KI = .02
'''
KP = 1.3
KD = 0.06
KI = .02
'''
#Vision speed 1.7
#normal 2.5
SPEED = 2.2
SPEED_INCREMENT = 1.2
SPEED_DEC =1.5
SPEED_MAX = SPEED
# MASTER = "Vision"
MASTER = "Laser"
#MASTER = "WALL"
class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan",LaserScan, self.reading, queue_size = 1)
        self.sub_ar = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar, queue_size = 1)
        self.sub_zed = rospy.Subscriber("/zed/rgb/image_rect_color",Image, self.reading_zed, queue_size = 1)
        #self.sub_controll = rospy.Subscriber("", ,self.contoller, queue_size=1)
        self.Target_distance = .6
        self.Previous_error = 0
        self.Data_size_front_S = 250
        
        self.Safe_distance = .45
        self.One_time = True
        self.Error_Integral = 0
        self.Direction = "Center"
        self.Vision_error = 0
        self.bridge = CvBridge()
        self.move = True
        self.NewTime = True
        self.NewMode = ""
        self.switchAction = 1000000000
    def contoller (self, click):
        if click:
            self.kill_click == True

    def ar (self, marker):
        global MASTER, SPEED, KP, KD , KI
        if len(marker.markers) > 0 :
            print "ENTERING", MASTER, SPEED, marker.markers[0].id, marker.markers[0].pose.pose.position.z
            if marker.markers[0].id == 20  and marker.markers[0].pose.pose.position.z < 0.5:
                if(self.NewTime):
                    self.switchAction = timer() + 1
                    self.NewTime = False
                    self.NewMode = "Vision"
                    print "Assinging new vision timer at: ", timer()
            elif marker.markers[0].id == 21 and marker.markers[0].pose.pose.position.z < .5:
                if(self.NewTime):
                    self.switchAction = timer() + 1.5
                    self.NewTime = False
                    self.NewMode = "Laser"
                    print "Assinging new vision timer at: ", timer()
            elif marker.markers[0].id == 19 and marker.markers[0].pose.pose.position.z < .9:
                MASTER = "19"
                KP = 1.2
                KD = 0
                KI = 0
                self.Test_error = 0
            elif  marker.markers[0].id == 23 and marker.markers[0].pose.pose.position.z < .7:
                MASTER = "WALL"
                KP = 1.2
                KD = .4
                KI = .02
                self.Test_error = 0
            elif marker.markers[0].id == 16 and marker.markers[0].pose.pose.position.z < .7:
                if(self.NewTime):
                    self.switchAction = timer() + 2.5
                    self.NewTime = False
                    self.NewMode = "Finish"
                    print "Assinging new vision timer at: ", timer()

        #print mrker.markers.id, marker.markers.pose.position.z
    def control(self, RD, LD, FD):
        #Use just one wall if there is just one wall
        if self.move:
            if self.Direction == "Right": 
                #Error = self.Test_error
                Error = self.Target_distance - RD
                move = True
            elif self.Direction == "Left":
                Error = LD - self.Target_distance
                move = True
            elif self.Direction == "Center":
                Error = self.Test_error
                move = True
            elif self.Direction == "Vision":
                Error = self.Vision_error 
                move = True
        else:   
            Error = 0
            move = False
            self.move = True

        Slope = (Error - self.Previous_error) / (self.currtm - self.prevtm)
        self.Error_Integral = (Error + self.Previous_error) / (self.currtm - self.prevtm)

        if self.Error_Integral > .1:
            self.Error_Integral = 0

        Target_angle = Error * KP + Slope * KD + self.Error_Integral * KI
        
        # Avoid wall crashing working on it
        saturation_value = .6
        global SPEED
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
        #print "Target angle = ", Target_angle
        #print "Target angle = ", Target_angle, " LD = ", LD, " RD = ", RD, " Target Distance = ", self.Target_distance 
        self.Previous_error = Error
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg = AckermannDrive()
        if MASTER == "Vision":
            SPEED = 1.25
        elif MASTER == "Laser" or MASTER == "WALL":
            SPEED = 2.2
        elif MASTER == "19":
            SPEED = 1        
        if -0.07 < Target_angle < 0.07 and FD > .8 and move:
            drive_msg.speed = SPEED * SPEED_INCREMENT
        else :
            drive_msg.speed = SPEED

        drive_msg.steering_angle = Target_angle
        drive_msg_stamped.drive = drive_msg
        #print Target_angle, LD, RD, self.Target_distance
        if move == False or self.kill_click:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0
            stop_msg.header.stamp = rospy.Time.now()
            self.kill_click == False
            self.pub.publish(stop_msg)

        else:
            self.pub.publish(drive_msg_stamped)


    def reading_zed(self, msg):
        if MASTER == "Vision":
            self.Direction = "Vision"
            frame = self.bridge.imgmsg_to_cv2(msg)
            # = self.bridge.imgmsg_to_cv2(debugImage, 'bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ymn = np.array([0,160,180])
            ymx = np.array([255,255,255])
            mask = cv2.inRange(hsv, ymn, ymx)
            res = cv2.bitwise_and(frame, frame, mask = mask)
            #cv2.imshow('mask', mask)

            width = len(mask[0])
            height = len(mask)
            # print "W =", width, " H = " , height
            threshhold = []
            # print "A", height/4 ,height*3/4, width
            for i in (239 ,479, 599, 719):
                for j in range(width):
                    if mask[i][j] == 255:
                        threshhold.append(j)
            if threshhold == []:
                Idex_average = 640
            else:
                Idex_average = np.average(np.array(threshhold))
            self.Vision_error = -(Idex_average - 640)/640
            
            if self.One_time:
                self.One_time = False
                self.currtm = time.time()
                self.prevtm = self.currtm

            self.currtm = time.time()
            self.control(0, 0, 0)
            self.prevtm = self.currtm        
        else:
            return

    def reading(self, msg):
        global MASTER
        if(self.switchAction < timer() and not self.NewTime):
            if self.NewMode == "Vision":
                MASTER = "Vision"
                KP = 1.2
                KD = .4
                KI = .0
                self.NewTime = True
                print "Switched with timer", timer()
            elif self.NewMode == "Laser":
                MASTER = "Laser"
                KP = 1.2
                KD = .4
                KI = .02
                self.NewTime = True
                self.Test_error = 0
            elif self.NewMode == "Finish":
                MASTER = "Finish"
        max_a = msg.angle_max
        min_a = msg.angle_min
        a_inc = msg.angle_increment
        Data_size_front = 100
        Data_size_left = 270
        Data_size_right = 270
        Angle_List = []
        Ranges_List = []
        if self.One_time:
            self.One_time = False
            self.currtm = time.time()
            self.prevtm = self.currtm
        #Create list of angles and ranges
        for i in range(60, len(msg.ranges)-60):
            if (i >= 535 and i <= 545) and msg.ranges[i] < self.Safe_distance :
                self.move = False
            Angle_List.append(a_inc * i + min_a) 
            Ranges_List.append(msg.ranges[i])    

        if MASTER == "Vision":
            self.Direction = "Vision"
            self.move = True
        elif MASTER == "Laser" or MASTER == "19" or MASTER == "WALL" :
            self.move = True
            #average
            Average = np.average(np.array(Ranges_List))
            Stand_deviation = np.std(np.array(Ranges_List))
            #higher value location
            threshold = []
            for i in range(len(Ranges_List)):
                if Ranges_List[i] > Average:
                    threshold.append(i)

            Idex_average = np.average(np.array(threshold))
            self.Test_error = (Idex_average - 460)/460

            if MASTER == "Laser" or MASTER == "19":
                self.Direction = "Center"
                self.currtm = time.time()
                self.control(0, 0, 0)
                self.prevtm = self.currtm
                #Filtering and dereminating the laser scaner
            else:
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
                if Right_distance > 1.3 and Right_distance > Left_distance: 
                    self.Direction = "Left"
                elif Left_distance > 1.3 and Right_distance < Left_distance:
                    self.Direction = "Right"

                self.currtm = time.time()
                self.control(Right_distance, Left_distance, Front_distance)
                self.prevtm = self.currtm  
        elif MASTER == "Finish":
            self.move = False
            self.currtm = time.time()
            self.control(0, 0, 0)
            self.prevtm = self.currtm
        else:
            return

if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow()
    rospy.spin()
