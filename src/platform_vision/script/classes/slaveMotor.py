#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
import math
<<<<<<< HEAD
import argparse
=======
>>>>>>> df78829fc5759e00b7c3023be6a30b0eac592d00

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

import os.path
import PNCC

VERBOSE = True
DEBUG = True

class SlaveCameraController:
    def __init__(self, activeTilitController=False):
        cv2.namedWindow('Slave Camera', cv2.WINDOW_NORMAL)
        self.leftMotorPub = rospy.Publisher('/right/pan/move', Float64, queue_size=2)
        self.left_image_sub = rospy.Subscriber('/stereo/left/image_raw', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/stereo/right/image_raw', Image, self.right_image_callback)
        self.bridge = CvBridge()

        self.fileExcite = False
        self.activeTilitController = activeTilitController
        if self.activeTilitController:
            self.slaveTiltMotorPub = rospy.Publisher('/right/tilt/move', Float64, queue_size=2)

        self.motorMinLimitTilt = -37
        self.motorMaxLimitTilt = 37

        self.left_image = None
        self.right_image = None
        self.savenumber = 0

        # Define the pyramid algorithm
        self.ScaleDown = ScaleDown
        if self.ScaleDown:
            self.imageSize = np.array([640, 420])
            self.templateSize = 51
            self.thresholdMotorController = np.array([20,6])
            pyramidLevel = 4
        else:
            self.imageSize = np.array([2048 , 1080])
            self.templateSize = 80
            self.thresholdMotorController = np.array([80,15])
            pyramidLevel = 7
        self.fastMatchingPyramid = PNCC.FastMatchingPyramid(self.imageSize, pyramidLevel=pyramidLevel,
                                                            windowSize=self.templateSize, grayImage = False,
                                                            showImage = True,drawDifferencesInImage= True,
                                                            operatingName = 'Slave ')

        self.exponatialGain = [0.0025, 0.0035]
        self.mapExponatialValue = [0.3, 0.35]
        self.motorMinLimit = -75
        self.motorMaxLimit = 75
        self.currentPos = [0.0, 0.0]
        self.stepDistance = 0.0001
        self.motorPos = [Float64(), Float64()]
        self.motorPos[0].data = self.currentPos[0]
        self.motorPos[1].data = self.currentPos[1]
        # set the motor to the zero position
        self.leftMotorPub.publish(self.motorPos[0])


    def __del__(self):
        for i in range(10):
            self.motorPos.data = 0.0
            self.motorPublisher.publish(self.motorPos)

    def saveImage(self, templateImage):
        self.savenumber += 1
        tempImgStr = '/home/abdulla/dev/Data/' + str(self.savenumber) + 'template.jpg'
        leftImgStr = '/home/abdulla/dev/Data/' + str(self.savenumber) + 'left.jpg'
        rightImgStr = '/home/abdulla/dev/Data/' + str(self.savenumber) + 'right.jpg'
        self.fileExcite = os.path.isfile(tempImgStr)
        while self.fileExcite:
            self.savenumber += 1
            tempImgStr = '/home/abdulla/dev/Data/' + str(self.savenumber) + 'template.jpg'
            self.fileExcite = os.path.isfile(tempImgStr)
            print (self.savenumber)

        cv2.imwrite(tempImgStr, templateImage)
        cv2.imwrite(rightImgStr, self.right_image)
        cv2.imwrite(leftImgStr, self.left_image)
        print ('Image saved')

    def my_mouse_callback(self, event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            self.saveImage(self.fastMatchingPyramid.getTemplate())
            pass

    def moveToZero(self):
        for i in range(10):
            self.motorPos[0].data = 0.0
            self.motorPos[1].data = 0.0
            self.currentPos = [0.0, 0.0]
            self.leftMotorPub.publish(self.motorPos[0])
            self.leftMotorPub.publish(self.motorPos[1])


    def moveMotor(self,value):
        speed = 0
        speed = np.sign(-value) * math.exp(abs(value)*self.exponatialGain[0])*self.mapExponatialValue[0]
        if abs(value) > self.thresholdMotorController[0] :
            self.currentPos[0] += speed
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            if self.currentPos[0] < self.motorMaxLimit and self.currentPos[0] > self.motorMinLimit :
                self.leftMotorPub.publish(self.motorPos[0])
        elif abs(value) <  self.thresholdMotorController[0] and abs(value) >  self.thresholdMotorController[1]:
            self.currentPos[0] -= value * 0.001
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            self.leftMotorPub.publish(self.motorPos[0])

        else:
            print "Pan Center Position"

    def TiltMoveMotor(self,value):
        TiltSpeed = 0
        TiltSpeed = np.sign(-value) * math.exp(abs(value)*self.exponatialGain[1])*self.mapExponatialValue[1]
        if abs(value) > self.thresholdMotorController[0] :
            self.currentPos[1] += TiltSpeed
            self.motorPos[1].data = self.currentPos[1]
            # print("Motor speed: ", self.TiltCurrentPos)
            if self.currentPos[1] < self.motorMaxLimitTilt and self.currentPos[1] > self.motorMinLimitTilt:
                self.slaveTiltMotorPub.publish(self.motorPos[1])
        elif abs(value) <  self.thresholdMotorController[0] and abs(value) >  self.thresholdMotorController[1]:
            self.currentPos[1] -= value * 0.001
            self.motorPos[1].data = self.currentPos[1]
            # print("Motor speed: ", self.TiltCurrentPos)
            self.slaveTiltMotorPub.publish(self.motorPos[1])
        else:
            print "Tilt Center Position"

    def convertROSToCV(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            if self.ScaleDown:
                return cv2.resize(cv_image, (self.imageSize[0], self.imageSize[1]))
            else:
                return cv_image
        except CvBridgeError, e:
            print e

    def left_image_callback(self, data):
        self.left_image = self.convertROSToCV(data)

    def right_image_callback(self, data):
        self.right_image = self.convertROSToCV(data)

    def trackObject(self):
        rate = rospy.Rate(60) # 10hz
        cv2.setMouseCallback('Slave Camera', self.my_mouse_callback)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.left_image is not None and self.right_image is not None:
                self.fastMatchingPyramid.createTemplate(self.left_image, self.imageSize/2)
                # cv2.imshow("template image", self.fastMatchingPyramid.getTemplate())
                centerPoint = self.fastMatchingPyramid.trackObject(self.right_image)
                differences = self.calculateDifferences(centerPoint)
                self.moveMotor(differences[0])
                if self.activeTilitController:
                    self.TiltMoveMotor(differences[1])
                ikey = cv2.waitKey(3)
                if ikey == ord('q'):
                    self.moveToZero()
                    exit()
                if ikey == ord('s'):
                    self.saveImage(self.fastMatchingPyramid.getTemplate())

    def calculateDifferences(self, centerPoint):
        if centerPoint is not None:
            return centerPoint - self.imageSize/2

        else:
            return np.array([0 , 0])

    def buildPyramid(self, image, maxleval):
        """Build image pyramid for level [0,...,maxlevel]
        """
        imgpyr = [image]
        aux = image
        for i in range(0,maxleval):
            aux = cv2.pyrDown(aux)
            imgpyr.append(aux)

        imgpyr.reverse()
        return imgpyr


ap = argparse.ArgumentParser(description='argument to control the slave controller!!')
ap.add_argument('-s', '--scale', default=False, help='This use to control the size of the image process')

args=vars(ap.parse_args())


def main(scale):
    rospy.init_node('FastMatchingPyramid', anonymous = True)
    slaveController = SlaveCameraController(activeTilitController=True, ScaleDown=scale)
    try:
        slaveController.trackObject()
        # rospy.spin()

    except KeyboardInterrupt:
        print "Shutting FastMatchingPyramid node down"
    cv2.destroyAllWindows()
    slaveController.moveToZero()

if __name__ == '__main__':
    scale = args['scale']
    print(scale)
    main(scale)
