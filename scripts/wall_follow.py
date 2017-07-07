#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan

KP = 0.4
KD = 0.3
SPEED = 1
SPEED_INCREMENT = 2.0

class WallFollow():
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size =1 )
        self.sub = rospy.Subscriber("/scan",LaserScan, self.reading, queue_size = 1)
        self.Target_distance = 0
        self.Previous_error = 0
        self.Data_size_front_S = 250
        self.Data_size_sides_S = 320
    
    def control(self, RD, LD, FD):

        Error = LD - RD
        #Use just one wall if there is just one wall
        if LD > 5: 
            Error = self.Target_distance - RD
        elif RD > 5:
            Error = LD - self.Target_distance
        else:
            if Error > 0:
                self.Target_distance = LD - Error
            else:
                self.Target_distance = LD + Error

        Slope = self.Previous_error - Error
        Target_angle = Error * KP + Slope * KD

        print Target_angle
        # Avoid wall crashing working on it
        if FD < 2.5:
            if Target_angle > 0:
                Target_angle += .5
            else:
                Target_angle -= .5

        self.Previous_error = Error

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
    # Sign depends on left or right (1 or -1)   
    # Right = 1
    # Left = -1
    def Average_Filter(self, Data_size, Start_index, ranges, angles, Sign):
        for i in range(Data_size):
            if i == 0:
                average = ranges[Start_index] * -np.sin(angles[Start_index]) * Sign
            else:
                Start_index += 1 * Sign
                average += ranges[Start_index] * -np.sin(angles[Start_index]) * Sign
            average = average/Data_size       
        return average              

    def reading(self, msg):
        #print msg.ranges
        max_a = msg.angle_max
        min_a = msg.angle_min
        a_inc = msg.angle_increment
        Safe_distance = .7
        Data_size_front = 250
        Data_size_sides = 320
        Data_size_left = 320
        Data_size_right = 320
        Angle_List = []

        #Create list of angles
        for i in range(len(msg.ranges)):
            Angle_List.append(a_inc * i + min_a) 
        '''
        #Filter  for the nex filter (no wall)
        for i in range(self.Data_size_sides_S):
            if i == 0:
                RD = int(((-np.pi/2 + max_a)/a_inc) - 70)
                LD = int(((np.pi/2 - max_a)/a_inc) - 70)
            else:
                RD += 1
                LD -= 1
            if msg.ranges[RD] > 9.9:
                del msg.ranges[RD]
                del Angle_List[RD]
                Data_size_right -= 1
            if msg.ranges[LD] > 9.9:
                del msg.ranges[LD]
                del Angle_List[LD]
                Data_size_left -= 1
        
        RD = int(((-np.pi/2 + max_a)/a_inc) - 70)
        LD = int(((np.pi/2 - max_a)/a_inc) - 70)
        FD = int(max_a/a_inc) - (Data_size_front/2)    
        # Sign depends on left or right (1 or -1)   
        # Right = 1
        # Left = -1
        Right_distance = self.Average_Filter(Data_size_right, RD, msg.ranges, Angle_List, 1)
        Left_distance = self.Average_Filter(Data_size_left, LD, msg.ranges, Angle_List, -1)
        Front_distance = self.Average_Filter(Data_size_front, FD, msg.ranges, Angle_List, 1)        
        '''
        # Trying to get an average of values insead of just 3 laser scans 
        #print a_inc
        for i in range(Data_size_sides):
            if i == 0:
                RD = int(((-np.pi/2 + max_a)/a_inc) - 70)
                LD = int(((np.pi/2 - max_a)/a_inc) - 70)
                FD = int(max_a/a_inc) - (Data_size_sides/2)
                Right_distance = msg.ranges[RD] * -np.sin(Angle_List[RD])
                Left_distance = msg.ranges[LD] * np.sin(Angle_List[LD])
                Front_distance = msg.ranges[FD]
                print Left_distance , Right_distance, Front_distance

            else:
                RD += 1
                LD -= 1
                FD += 1
                print RD , LD, FD
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

        Right_distance = Right_distance/Data_size_right
        Left_distance = Left_distance/Data_size_left
        Front_distance = Front_distance/Data_size_front

        self.control(Right_distance, Left_distance, Front_distance)
        #print Left_distance , Right_distance, Front_distance
        #print Angle_List



if __name__=="__main__":
    rospy.init_node("wall_follow")
    WallFollow()
    rospy.spin()