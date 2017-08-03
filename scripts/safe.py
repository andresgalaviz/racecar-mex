#!/usr/bin/python
 
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
 
class SafetyControllerNode:
    def __init__(self):
        # subscribe to incomming Ackermann drive commands
        rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
 
        # publisher for the safe Ackermann drive command
        self.cmd_pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=10)

        self.DRIVE = True
        self.DANGER_DISTANCE = .5 #meters
        self.ANGLE = 40*np.pi/180 #degrees to radians

    def laser_callback(self,msg):
        #Copy over to local variables
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges

        for index in range(0,len(ranges)):
            angle = angle_min + angle_inc*index

            LOW = -self.ANGLE/2
            HIGH = self.ANGLE/2

            if angle > LOW and angle < HIGH:
                #We are looking in the are in front of us
                if ranges[index] < self.DANGER_DISTANCE:
                    self.DRIVE = False
                    self.ackermann_cmd_input_callback(AckermannDriveStamped())
                    break

        #WE didnt find a obstacle, keep driving!!!
        self.DRIVE = True


    def ackermann_cmd_input_callback(self, msg):
        # republish the input as output (not exactly "safe")
        if self.DRIVE:
            self.cmd_pub.publish(msg)
        else:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0
            stop_msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(stop_msg)
 
if __name__ == "__main__":
    rospy.init_node("safety_controller")
    node = SafetyControllerNode()
    rospy.spin()
