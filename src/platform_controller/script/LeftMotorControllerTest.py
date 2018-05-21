#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
import math
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16 , Int64

#import Libray
import trackByAruco
import trackByColor
import PNCC

VERBOSE = True
DEBUG = True


class MasterCameraController:
    def __init__(self):
        cv2.namedWindow('Master Camera', cv2.WINDOW_NORMAL)
        self.imageSize = np.array([2048 , 1080])
        self.algorithmForTracking = algorithmForTracking
        self.suspendMotor = suspendMotor
        self.independedTiltMotor = True



        #Motor Controller
        self.motorPublisher = rospy.Publisher("/left/pan/move", Float64, queue_size=10, latch=True)


        self.exponatialGain = [0.003, 0.0035] # pan, tilt
        self.mapExponatialValue = [0.2, 0.35]
        self.motorMinLimit = -75
        self.motorMaxLimit = 75
        self.motorMinLimitTilt = -30
        self.motorMaxLimitTilt = 30

        self.currentPos = [0.0, 0.0]
        self.stepDistance = 0.0001
        self.motorPos = [Float64(), Float64()]
        self.motorPos[0].data = self.currentPos[0]
        self.motorPos[1].data = self.currentPos[1]
        # set the motor to the zero position
        self.motorPublisher.publish(self.motorPos[0])
        self.tiltMotorPublisher.publish(self.motorPos[1])

    def testSpeed(self):
        for i in range(400):
            speed = i * 10
            self.motorPublisher.publish(self.motorPos[0])
