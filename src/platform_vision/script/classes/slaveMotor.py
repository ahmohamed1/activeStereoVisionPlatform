#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
import math

import argparse


from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int64
from std_msgs.msg import Float64

import os.path
import PNCC
import BaseFeatureMatching
import vergincyDepthClass

VERBOSE = True
DEBUG = True

class SlaveCameraController:
    def __init__(self, activeTilitController=False,algorithmToUse= 'PNCC', scaleDown = 0):

        self.drawTrackingSystem = vergincyDepthClass.DrawTrackingSystem()


        cv2.namedWindow('Slave Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Slave Camera', (900,600))
        self.leftMotorPub = rospy.Publisher('/right/pan/move', Float64, queue_size=2)
        self.left_image_sub = rospy.Subscriber('/stereo/left/image_raw', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/stereo/right/image_raw', Image, self.right_image_callback)
        self.templateSizeSub = rospy.Subscriber('/templateSize', Int64, self.templateSizeCallBack)
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
        self.ScaleDown = scaleDown
        if self.ScaleDown:
            # self.imageSize = np.array([920, 640])
            self.imageSize = np.array([2048/2 , 1080/2])
            self.templateSize = 35
            self.thresholdMotorController = np.array([20,6])
            pyramidLevel = 4
            self.scaleTemplate = 0.5
        else:
            self.imageSize = np.array([2048 , 1080])
            self.templateSize = 121
            self.thresholdMotorController = np.array([50,10])
            pyramidLevel = 7
            self.scaleTemplate = 1.0
        self.algorithmToUse = algorithmToUse
        self.featueMatchingAlgorithmState = False
        if self.algorithmToUse == 'PNCC':
            self.fastMatchingPyramid = PNCC.FastMatchingPyramid(self.imageSize, pyramidLevel=pyramidLevel,
                                                            windowSize=self.templateSize, grayImage = False,
                                                            showImage = True,drawDifferencesInImage= True,
                                                            operatingName = 'Slave ')
        elif self.algorithmToUse == 'kaze' or self.algorithmToUse == 'FLANN' or self.algorithmToUse == 'Brute':
            self.featueMatchingAlgorithmState = True
            self.trackingFeature = BaseFeatureMatching.BaseFeatureMatching()


        self.exponatialGain = [0.0025, 0.0035]
        self.mapExponatialValue = [0.3, 0.35]
        self.motorMinLimit = -75
        self.motorMaxLimit = 75
        self.currentPos = [-5.0, -6.0]
        self.stepDistance = 0.0001
        self.motorPos = [Float64(), Float64()]
        i = 0
        r = rospy.Rate(10) # 10hz
        while (i < 5):
            self.motorPos[0].data = self.currentPos[0]
            self.motorPos[1].data = self.currentPos[1]
            # set the motor to the zero position
            self.leftMotorPub.publish(self.motorPos[0])
            self.slaveTiltMotorPub.publish(self.motorPos[1])
            r.sleep()
            i +=1
        # sleep for 0.5 seconds
        rospy.sleep(.5)
        self.terminateButton = 1

    def __del__(self):
        for i in range(10):
            self.currentPos = [0.0, 0.0]
            self.leftMotorPub.publish(self.motorPos[0])
            self.slaveTiltMotorPub.publish(self.motorPos[1])


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
            if self.algorithmToUse == 'PNCC':
                self.saveImage(self.fastMatchingPyramid.getTemplate())
            pass
        if event==cv2.EVENT_RBUTTONDOWN:
            self.terminateButton += 1

    def moveToZero(self):
        print('Motor move to ZERO position!!!')
        for i in range(10):
            self.motorPos[0].data = 0.0
            self.motorPos[1].data = 0.0
            self.currentPos = [0.0, 0.0]
            # set the motor to the zero position
            self.leftMotorPub.publish(self.motorPos[0])
            self.slaveTiltMotorPub.publish(self.motorPos[1])



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

    def templateSizeCallBack(self, data):
        templateSize = data.data
        if self.algorithmToUse == 'PNCC':
            self.fastMatchingPyramid.setTemplateSize(int((templateSize*self.scaleTemplate)/1.8))


    def computeTheCenterUsingDifferentAlgorithm(self, template,image):
        centerPoint = None
        if self.algorithmToUse == 'PNCC':
            self.fastMatchingPyramid.createTemplate(template, self.imageSize/2)
            # cv2.imshow("template image", self.fastMatchingPyramid.getTemplate())
            _img, centerPoint = self.fastMatchingPyramid.trackObject(image)
            cv2.imshow('Slave Camera', _img)
        elif self.algorithmToUse == 'feature' or self.featueMatchingAlgorithmState:
            Size = self.templateSize
            template = template[self.imageSize[1]/2-Size:self.imageSize[1]/2+Size, self.imageSize[0]/2-Size:self.imageSize[0]/2+Size ]
            # _img, centerPoint = self.trackingFeature.BruteForceMatchingwithSIFTDescriptorsandRatioTest(template, image)
            _img, centerPoint = self.trackingFeature.algorithmDictionary[self.algorithmToUse](template, image) # kaze, FLANN, Brute
            # _img, centerPoint = self.trackingFeature.kaze_match(template, image)
            cv2.imshow('Slave Camera', _img)
        return centerPoint
    def trackObject(self):
        rate = rospy.Rate(60) # 10hz
        cv2.setMouseCallback('Slave Camera', self.my_mouse_callback)
        while not rospy.is_shutdown():
            rate.sleep()
            # Publish the coordinate
            self.drawTrackingSystem.calculateThePosition()
            if self.left_image is not None and self.right_image is not None:
                centerPoint = self.computeTheCenterUsingDifferentAlgorithm(self.left_image,self.right_image )
                differences = self.calculateDifferences(centerPoint)
                self.moveMotor(differences[0])
                if self.activeTilitController:
                    self.TiltMoveMotor(differences[1])
                ikey = cv2.waitKey(3)
                if ikey == ord('q'):
                    self.moveToZero()
                    exit()
                elif ikey == ord('s'):
                    if self.algorithmToUse == 'PNCC':
                        self.saveImage(self.fastMatchingPyramid.getTemplate())
                if self.terminateButton == 2 :
                    break
                # if self.algorithmToUse == 'PNCC':
                #     if self.fastMatchingPyramid.getTerminatedState():
                #         break

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
ap.add_argument('-s', '--scale',type=int, default=0, help='This use to control the size of the image process')
ap.add_argument('-a', '--algorithm', default= 'PNCC', help='Chosse the algorithm to use PNCC, kaze, FLANN, Brute ')

args=vars(ap.parse_args())

ScaleDown = args['scale']
algorithm = args['algorithm']
def main(scale):
    rospy.init_node('SlaveMotorController', anonymous = True)
    slaveController = SlaveCameraController(activeTilitController=True, algorithmToUse = algorithm, scaleDown=ScaleDown)
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
