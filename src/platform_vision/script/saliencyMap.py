#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
import sys
import numpy as np
import math
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16

import pySaliencyMap

VERBOSE = True
DEBUG = True

class ControlMotors:

    def __init__(self):

        #Define the saliency class
        self.smallimageSize = np.array([640 , 420])
        self.sm = pySaliencyMap.pySaliencyMap(640, 420)
        self.TrackingData = []
        self.oldTime = 0.0
        self.timeDiff = 0.0
        self.oldNumber = 0
        self.sameNumber = 0
        self.saveDataAble = True
        # Define the publisher to the left motor
        # self.leftMotorPub = rospy.Publisher('/left/pan/move', Vector3, queue_size=1)
        self.image_sub = rospy.Subscriber('/stereo/left/image_color', Image, self.image_callback, queue_size=1) #/stereo/right/image_color

        self.bridge = CvBridge()
        self.image = 0
        self.image_state = False

        # This virable use to filter the

        self.imageSize = np.array([2048 , 1080])
        # self.imageSize = np.array([640 , 480])
        self.templateSize = 200
        self.x1 = (self.imageSize[0]/2 - (self.templateSize/2))
        self.x2 = (self.imageSize[0]/2 + (self.templateSize/2))
        self.y1 = (self.imageSize[1]/2 - (self.templateSize/2))
        self.y2 = (self.imageSize[1]/2 + (self.templateSize/2))

        #Motor Controller

        self.motorPublisher = rospy.Publisher("/left/pan/move", Float64, queue_size=10, latch=True)
        self.tiltMotorPublisher = rospy.Publisher("/tilt_both_motor/move", Float64, queue_size=10, latch=True)

        # self.poisitionSubscriper = rospy.Subscriber("/position", Float64, self.positionCallback)
        self.exponatialGain = [0.003, 0.0035]
        self.mapExponatialValue = [0.2, 0.35]
        self.motorMinLimit = -120
        self.motorMaxLimit = 120
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

    def __del__(self):
        for i in range(10):
            self.motorPos.data = 0.0
            self.motorPublisher.publish(self.motorPos)

    def moveToZero(self):
        # self.saveDateAfterFinish()
        for i in range(10):
            self.motorPos[0].data = 0.0
            self.motorPos[1].data = 0.0
            self.currentPos = [0.0, 0.0]
            self.motorPublisher.publish(self.motorPos[0])
            self.tiltMotorPublisher.publish(self.motorPos[1])
            self.TrackingData = []
            self.oldTime = 0.0
            self.timeDiff = 0.0

    def computeSalency(self, frame):
        # computation
        frame = cv2.resize(frame, (self.smallimageSize[0], self.smallimageSize[1]))
        saliency_map = self.sm.SMGetSM(frame)
        saliency_map = cv2.resize(saliency_map, (self.imageSize[0], self.imageSize[1]))
        self.createWindows('Saliency map', saliency_map)
        # salient_region = self.sm.SMGetSalientRegion(frame)
        # self.createWindows('Salient region', salient_region)
        img = cv2.convertScaleAbs(saliency_map*255)
        # _, thresholdImage = cv2.threshold(img,hul,huh,cv2.THRESH_BINARY)
        self.createWindows('threshold', img)
        return saliency_map

    def convertROSToCV(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            return cv_image, True
        except CvBridgeError, e:
            print e

    def image_callback(self, data):
        self.image, self.image_state = self.convertROSToCV(data)

    def trackObject(self):
        rate = rospy.Rate(15) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            if self.image_state is True:
                ## call the function
                self.computeSalency(self.image)
                # centerPoint = self.findAurco(self.image)
                # centerPoint = self.processImageBasedColor.find_countor(self.image)
                # differences = self.calculateDifferences(centerPoint)
                # print ("diff: i%", differences)
                # self.moveMotor(differences[0])
                # self.TiltMoveMotor(differences[1])
                ikey = cv2.waitKey(3)
                if ikey == ord('q') : #or (self.sameNumber == 15 and self.saveDataAble == True):
                    # self.PublishMotorAtSpeedRate(0, 0)
                    self.sameNumber = 0
                    break

    def calculateDifferences(self, centerPoint):
        if centerPoint is not None:
            return  centerPoint - self.imageSize/2

        else:
            return np.array([0 , 0])

    def returnCenterOfArcuo(self, markerCorners):
        centerPoint = np.array([0, 0])
        if len(markerCorners) > 0:
            centerPoint[0] = int((markerCorners[0][0][1][0] + markerCorners[0][0][2][0] + markerCorners[0][0][3][0] + markerCorners[0][0][0][0])/4)
            centerPoint[1] = int((markerCorners[0][0][1][1] + markerCorners[0][0][2][1] + markerCorners[0][0][3][1] + markerCorners[0][0][0][1])/4)
        else:
            centerPoint = self.imageSize/2
        return centerPoint

    def moveMotor(self,value):
        speed = 0
        speed = np.sign(-value) * math.exp(abs(value)*self.exponatialGain[0])*self.mapExponatialValue[0]
        if abs(value) > 80 :
            self.currentPos[0] += speed
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            if self.currentPos[0] < self.motorMaxLimit and self.currentPos[0] > self.motorMinLimit :
                self.motorPublisher.publish(self.motorPos[0])
        elif abs(value) <  80 and abs(value) >  20:
            self.currentPos[0] -= value * 0.0025
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            self.motorPublisher.publish(self.motorPos[0])

        else:
            print "Pan Center Position"

    def TiltMoveMotor(self,value):
        TiltSpeed = 0
        TiltSpeed = np.sign(-value) * math.exp(abs(value)*self.exponatialGain[1])*self.mapExponatialValue[1]
        if abs(value) > 40 :
            self.currentPos[1] += TiltSpeed
            self.motorPos[1].data = self.currentPos[1]
            # print("Motor speed: ", self.TiltCurrentPos)
            if self.currentPos[1] < self.motorMaxLimitTilt and self.currentPos[1] > self.motorMinLimitTilt :
                self.tiltMotorPublisher.publish(self.motorPos[1])
        elif abs(value) <  40 and abs(value) >  15:
            self.currentPos[1] -= value * 0.0025
            self.motorPos[1].data = self.currentPos[1]
            # print("Motor speed: ", self.TiltCurrentPos)
            self.tiltMotorPublisher.publish(self.motorPos[1])
        else:
            print "Tilt Center Position"



    def createWindows(self, imageName, imageToShow, WindowSize = (960,640)):
        cv2.namedWindow(imageName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(imageName, WindowSize)
        cv2.imshow(imageName, imageToShow)

    def storeData(self,speed, diff):
        if self.oldTime == 0:
            self.timeDiff = 0
            time = rospy.get_time()
            self.oldTime = time
        else:
            time = rospy.get_time()
            self.timeDiff = (time - self.oldTime)+ self.timeDiff
            self.oldTime = time
        self.TrackingData.append([self.timeDiff, speed, diff])


    def saveDateAfterFinish(self):
        import csv
        import os
        name = '/home/abdulla/TrackingSpeed/speedTest.csv'
        fileBool = os.path.isfile(name)    # False
        if fileBool == False:
            with open(name, "wb") as f:
                writer = csv.writer(f)
                writer.writerows(self.TrackingData)

def main(args):
    rospy.init_node('ControlMasterMotor', anonymous = True)
    controlMotors = ControlMotors()
    try:
        controlMotors.trackObject()

    except KeyboardInterrupt:
        print "Shutting ControlMasterMotor node down"
    cv2.destroyAllWindows()
    controlMotors.moveToZero()

if __name__ == '__main__':
    main(sys.argv)
