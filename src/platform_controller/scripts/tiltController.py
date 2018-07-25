#!/usr/bin/env python

import rospy
import sys
import numpy as np
import math

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

PI = 3.14159265359

class TiltController:

    def __init__(self):

        # Create the Subscriber recive degree and publish it to motor in radians
        self.leftMotorSubscriber = rospy.Subscriber("/tilt_controller/state", JointState, self.leftAngleCallback)

        # Initialize the motor publisher
        self.leftMotorPublisher = rospy.Publisher("/tilt_controller/command", Float64, queue_size=5, latch=True)
        self.start = 0.0
        self.motorPublish = Float64()
        self.currentPosition = 1.65
        self.motorPublish.data = self.currentPosition
        self.leftMotorPublisher.publish(self.motorPublish)
        self.state = True
        rospy.sleep(2)

    def deg2rad(self, value):
        return value * PI / 180

    def leftAngleCallback(self, msg):
        self.currentPosition = msg.current_pos
        print (self.currentPosition)

    def startLoop(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

            if self.currentPosition >= 1.6 :
                self.motorPublish.data = -1.65
                self.leftMotorPublisher.publish(self.motorPublish)

            elif self.currentPosition <= -1.6:
                self.motorPublish.data = 1.65
                self.leftMotorPublisher.publish(self.motorPublish)
            else:
                print ("Do nothing")



def main():
    rospy.init_node('TiltController', anonymous = True)
    tiltController = TiltController()
    try:
        tiltController.startLoop()
        # rospy.spin()

    except KeyboardInterrupt:
        print "Shutting ArucoFinder node down"

if __name__ == '__main__':
    main()
