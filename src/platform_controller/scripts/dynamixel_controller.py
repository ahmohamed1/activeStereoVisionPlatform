#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
from std_msgs.msg import Float64
import math

class MotorController:

    def __init__(self):
        self.motorPublisher = rospy.Publisher("/pan_left_controller/command", Float64, queue_size=10, latch=True)

        self.poisitionSubscriper = rospy.Subscriber("/position", Float64, self.positionCallback)

        self.motorMinLimit = -1.5
        self.motorMaxLimit = 1.5
        self.currentPos = 0.0
        self.stepDistance = 0.1
        self.motorPos = Float64()
        self.motorPos.data = self.currentPos

        # set the motor to the zero position
        self.motorPublisher.publish(self.motorPos)

    def moveMotor(self,value):
        if value < 0 :
            self.currentPos += self.stepDistance
            self.motorPos.data = self.currentPos

            if self.currentPos < self.motorMaxLimit and self.currentPos > self.motorMinLimit :
                self.motorPublisher.publish(self.motorPos)

        elif value > 0 :
            self.currentPos -= self.stepDistance
            self.motorPos.data = self.currentPos
            if self.currentPos < self.motorMaxLimit and self.currentPos > self.motorMinLimit :
                self.motorPublisher.publish(self.motorPos)
        else:
            print "Center Position"

    def positionCallback(self, msg):
        self.moveMotor(msg.data)

def main(args):
    rospy.init_node('dynamixelController', anonymous = True)
    motorController = MotorController()
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print "Shutting FastMatchingPyramid node down"


if __name__ == '__main__':
    main(sys.argv)
