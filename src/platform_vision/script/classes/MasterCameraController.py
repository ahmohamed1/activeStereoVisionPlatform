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
import pySaliencyMap
VERBOSE = True
DEBUG = True


class MasterCameraController:
    def __init__(self, algorithmForTracking, suspendMotor = True ,saveData=False, ScaleDown=False):
        cv2.namedWindow('Master Camera', cv2.WINDOW_NORMAL)
        self.ScaleDown = ScaleDown
        if self.ScaleDown:
            self.imageSize = np.array([ int(2048/2), int(1080/2)])
            self.thresholdMotorController = np.array([20,5])
            self.pyramidLayer = 3
        else:
            self.imageSize = np.array([2048 , 1080])
            self.thresholdMotorController = np.array([80,12])
            self.pyramidLayer = 7

        self.algorithmForTracking = algorithmForTracking
        self.suspendMotor = suspendMotor
        self.independedTiltMotor = True
        # self.incremantel = 0
        print (algorithmForTracking)
        if self.algorithmForTracking == 'color':
            print ('Running color')
            self.processImageBasedColor = trackByColor.ProcessImageBasedColor(self.imageSize)
        elif self.algorithmForTracking == 'aruco':
            self.arucoTracking = trackByAruco.TrackByAruco(self.imageSize)
            self.ideaToTrackSub = rospy.Subscriber('/ideaNumber',Int64, self.ideaToTrackCallback, queue_size=1)
            self.ideatToTrack = 1
        elif self.algorithmForTracking == 'saliency':
            self.saliencyMap = pySaliencyMap.pySaliencyMap(self.imageSize[0], self.imageSize[1])

        elif self.algorithmForTracking == 'colorWithPNCC':
            self.processImageBasedColor = trackByColor.ProcessImageBasedColor(self.imageSize, True)
            self.PNCC = PNCC.FastMatchingPyramid(self.imageSize, pyramidLevel = self.pyramidLayer, windowSize = 150,
                                                 grayImage = False , showImage = True,drawDifferencesInImage= False,
                                                 operatingName = 'Master  ')

            self.pyramidPNCCState = True
            self.TemplateList = []

        elif self.algorithmForTracking == 'PNCC':
            self.PNCC = PNCC.FastMatchingPyramid(self.imageSize, pyramidLevel = self.pyramidLayer, windowSize = 51,
                                                grayImage = False, showImage = True,drawDifferencesInImage= False,
                                                operatingName = 'Master  ')

            self.TemplateCenter = np.array([0,0])
            cv2.resizeWindow('Master Camera', (900,600))


        self.saveDataAble = saveData
        if self.saveDataAble == True:
            self.saveData = SaveData()
        # Define the publisher to the left motor
        self.image_sub = rospy.Subscriber('/stereo/left/image_color', Image, self.image_callback, queue_size=1) #/stereo/right/image_color
        self.bridge = CvBridge()
        self.image = None
        # Publish the template Size limit
        self.templateSizePub = rospy.Publisher('/templateSize', Int64, queue_size=5)
        self.OldTemplateSize = 0
        self.templateSizeLimits = 20
        #Motor Controller
        self.motorPublisher = rospy.Publisher("/left/pan/move", Float64, queue_size=10, latch=True)
        if self.independedTiltMotor:
            self.tiltMotorPublisher = rospy.Publisher("/left/tilt/move", Float64, queue_size=10, latch=True)
        else:
            self.tiltMotorPublisher = rospy.Publisher("/tilt_both_motor/move", Float64, queue_size=10, latch=True)

        self.exponatialGain = [0.003, 0.0035] # pan, tilt
        self.mapExponatialValue = [0.2, 0.35]
        # self.mapExponatialValue = [10, 0.35]
        self.motorMinLimit = -75
        self.motorMaxLimit = 75
        self.motorMinLimitTilt = -37
        self.motorMaxLimitTilt = 37

        self.currentPos = [0.0, 0.0]
        self.stepDistance = 0.0001
        self.motorPos = [Float64(), Float64()]
        self.motorPos[0].data = self.currentPos[0]
        self.motorPos[1].data = self.currentPos[1]
        # set the motor to the zero position
        # self.motorPublisher.publish(self.motorPos[0])
        # self.tiltMotorPublisher.publish(self.motorPos[1])
        self.terminateButton = 0
    def __del__(self):
        for i in range(10):
            self.motorPos.data = 0.0
            self.motorPublisher.publish(self.motorPos)

    def moveToZero(self):
        # self.saveDateAfterFinish()
        for i in range(50):
            self.motorPos[0].data = 0.0
            self.motorPos[1].data = 0.0
            self.currentPos = [0.0, 0.0]
            self.motorPublisher.publish(self.motorPos[0])
            self.tiltMotorPublisher.publish(self.motorPos[1])
            self.TrackingData = []
            self.oldTime = 0.0
            self.timeDiff = 0.0

    def my_mouse_callback(self, event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            print x,y
            self.TemplateCenter = np.array([x , y])

        if event==cv2.EVENT_RBUTTONDOWN:
            self.terminateButton += 1

    def ideaToTrackCallback(self,data):
        self.ideatToTrack = data.data

    def convertROSToCV(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            if self.ScaleDown:
                return cv2.resize(cv_image, (self.imageSize[0], self.imageSize[1]))
            else:
                return cv_image

        except CvBridgeError, e:
            print e

    def image_callback(self, data):
        self.image = self.convertROSToCV(data)
        # print(self.image.shape)

    def publishTemplateSize(self,size):
        if size > self.OldTemplateSize + self.templateSizeLimits or size < self.OldTemplateSize - self.templateSizeLimits :
            templateSize = Int64()
            templateSize.data = size
            self.templateSizePub.publish(templateSize)
            self.OldTemplateSize = size

    def compute_avarge_around_most_interest(self, img, point, window_size=15):
        rectangule = img[point[0]-window_size:point[0]+window_size,point[1]-window_size:point[1]+window_size ]
        mean = rectangule.mean()
        return mean

    def processSaliencyMap(self,image):
        # #get all the matches:
        old_circles = None
        maximumList = []
        meanList = []
        maximum_value = 0
        i = 0
        for i in range(5):
        # while (maximum_value < 150):
            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(image)
            cv2.circle(image, maxLoc, 45, (0, 0, 0), -1)
        #     cv2.circle(img, maxLoc, 5, (255, 255, 0), 2)
            mean = self.compute_avarge_around_most_interest(image, maxLoc, 50)
            print("maximum value: ", maxVal," location: ", maxLoc, " Mean:", mean)
            maximumList.append((str(i),maxVal))
            meanList.append(((str(i),mean),maxLoc))
            i += 1
            maximum_value = maxVal
            if old_circles is not None:
                cv2.line(image, old_circles, maxLoc, (255,255,0))
                old_circles = maxLoc
            else:
                old_circles = maxLoc
            cv2.imshow('saliencylines', image)
            cv2.waitKey(1)

    def trackObjectInImage(self, image):
        centerPoint = None
        if self.algorithmForTracking == 'color':
            Listoutput = self.processImageBasedColor.trackObject(image)
            centerPoint = Listoutput[0]
        elif self.algorithmForTracking == 'aruco':
            self.arucoTracking.setIdeaToTrack(self.ideatToTrack)
            centerPoint = self.arucoTracking.trackObject(image)
            self.publishTemplateSize(int(self.arucoTracking.getTemplateSize()))
        elif self.algorithmForTracking == 'colorWithPNCC':
            centerPoint = self.trackObjectUsingPNCCandColor(image)
        elif self.algorithmForTracking == 'PNCC':
            if self.TemplateCenter[0] != 0:
                self.PNCC.createTemplate(image, self.TemplateCenter)
                self.TemplateCenter = np.array([0, 0])
            centerPoint = self.PNCC.trackObject(image)
            lineSize = 20
            imageToShow = cv2.line(image,(self.imageSize[0]/2, self.imageSize[1]/2-lineSize),(self.imageSize[0]/2, self.imageSize[1]/2+lineSize),(0,0,255),2)
            imageToShow = cv2.line(image,(self.imageSize[0]/2-lineSize, self.imageSize[1]/2),(self.imageSize[0]/2+lineSize, self.imageSize[1]/2),(0,0,255),2)
            cv2.imshow('Master Camera', image)
        elif self.algorithmForTracking == 'saliency':
            img = cv2.resize(image, (640,420))
            saliency_img = self.saliencyMap.SMGetSM(img)
            corrected_image = cv2.convertScaleAbs(saliency_img*255)
            color_image = cv2.applyColorMap(corrected_image, cv2.COLORMAP_JET)
            cv2.imshow('saliency color', color_image)
            cv2.imshow('saliency', corrected_image)
            self.processSaliencyMap(corrected_image)
            # cv2.imshow('saliency canny', img)
            cv2.waitKey(3)

        # Return the coordinate
        return self.calculateDifferences(centerPoint)


    def trackObjectUsingPNCCandColor(self,img):
        image = img
        # Check if there paryimad algorithm complete
        if self.pyramidPNCCState == True:
            centerOfTarget = self.processImageBasedColor.trackObject(image)
            print centerOfTarget
            if len(self.TemplateList) == 0:
                # crop the image using the coordinate from the process before
                self.PNCC.createTemplate(img, centerOfTarget[0])
                self.TemplateList.append(self.PNCC.getTemplate())
            else:
                foundTemplateList = self.PNCC.createMultipleTemplate(image, centerOfTarget)
                #Compare the old template with the new list and chose the one with less similarity
                similarity = 1
                oldTemplate = self.PNCC.getTemplate()
                # print (len(foundTemplateList))
                # for oldTemplate in self.TemplateList:
                for temp in foundTemplateList:
                    if temp.shape == oldTemplate.shape:
                        s = self.PNCC.compareTemplate(temp, oldTemplate)
                        print s
                        if s < similarity:
                            similarity = s
                            self.PNCC.setTemplate(temp)
            # Return the PNCCState to False
                print similarity
                self.TemplateList.append(self.PNCC.getTemplate())
            cv2.imshow('Template', self.PNCC.getTemplate())
            cv2.waitKey(0)
            self.pyramidPNCCState = False

        if self.pyramidPNCCState == False:
            centerPoint = self.PNCC.trackObject(img)
            # self.incremantel += 1
            # print self.incremantel
        return centerPoint


    def trackObject(self):
        rate = rospy.Rate(15) # 10hz
        cv2.setMouseCallback('Master Camera', self.my_mouse_callback)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.image is not None:
                ## call the function
                differences = self.trackObjectInImage(self.image)
                print ("diff: i%", differences)

                if self.suspendMotor:
                    self.moveMotor(differences[0])
                    # self.moveMotorSpeed(differences[0])
                    self.TiltMoveMotor(differences[1])
                ikey = cv2.waitKey(3)
                # self.createWindows('image', self.image)
                if self.terminateButton == 2 :
                    break
                if ikey == ord('q') : #or (self.sameNumber == 15 and self.saveDataAble == True):
                    # self.self.saveData.sameNumber = 0
                    break
                elif ikey == ord('n'):
                    print ("Next object")
                    self.pyramidPNCCState = True
                    self.incremantel = 0
                # if self.incremantel == 20:
                #     print ("Next object")
                #     self.pyramidPNCCState = True
                #     self.incremantel = 0
            else:
                print("No Image To Process!!!")

    def createWindows(self, imageName, imageToShow, WindowSize = (640,320)):
        cv2.namedWindow(imageName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(imageName, WindowSize)
        cv2.imshow(imageName, imageToShow)
        cv2.waitKey(10)

    def calculateDifferences(self, centerPoint):
        if centerPoint is not None:
            return  centerPoint - self.imageSize/2
        else:
            return np.array([0 , 0])

    def moveMotor(self,value):
        speed = 0
        speed = np.sign(-value) * math.exp(abs(value)*self.exponatialGain[0])*self.mapExponatialValue[0]
        if abs(value) > self.thresholdMotorController[0] :
            self.currentPos[0] += speed
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            if self.currentPos[0] < self.motorMaxLimit and self.currentPos[0] > self.motorMinLimit :
                self.motorPublisher.publish(self.motorPos[0])
        elif abs(value) <  self.thresholdMotorController[0] and abs(value) >  self.thresholdMotorController[1]:
            self.currentPos[0] -= value * 0.001
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            self.motorPublisher.publish(self.motorPos[0])
        else:
            print "Pan Center Position"
        # Save the speed
        if self.saveDataAble == True:
            self.saveData.storeData(speed, value)
            self.saveData.checkSpeedRepetedlly(value)

    # def moveMotorSpeed(self, value):
    #     speed = 0
    #     speed = np.sign(value) * math.exp(abs(value)*self.exponatialGain[0])*self.mapExponatialValue[0]
    #     if abs(speed) > 20 :
    #         self.currentPos[0] = speed
    #         self.motorPos[0].data = self.currentPos[0]
    #         print("Motor speed: ", self.currentPos)
    #         # if self.currentPos[0] < self.motorMaxLimit and self.currentPos[0] > self.motorMinLimit :
    #         self.motorPublisher.publish(self.motorPos[0])
    #     elif abs(speed) <  20 and abs(speed) >  10:
    #         self.currentPos[0] = 80
    #         self.motorPos[0].data = self.currentPos[0]
    #         print("Motor speed: ", self.currentPos)
    #         self.motorPublisher.publish(self.motorPos[0])
    #     else:
    #         print "Pan Center Position"

    def TiltMoveMotor(self,value):
        TiltSpeed = 0
        if self.independedTiltMotor:
            TiltSpeed = np.sign(value) * math.exp(abs(value)*self.exponatialGain[1])*self.mapExponatialValue[1]
        else:
            TiltSpeed = np.sign(-value) * math.exp(abs(value)*self.exponatialGain[1])*self.mapExponatialValue[1]

        if abs(value) > self.thresholdMotorController[0] :
            self.currentPos[1] += TiltSpeed
            self.motorPos[1].data = self.currentPos[1]
            # print("Motor speed: ", self.TiltCurrentPos)
            if self.currentPos[1] < self.motorMaxLimitTilt and self.currentPos[1] > self.motorMinLimitTilt :
                self.tiltMotorPublisher.publish(self.motorPos[1])
        elif abs(value) <  self.thresholdMotorController[0] and abs(value) >  self.thresholdMotorController[1]:
            self.currentPos[1] += value * 0.001
            self.motorPos[1].data = self.currentPos[1]
            # print("Motor speed: ", self.TiltCurrentPos)
            self.tiltMotorPublisher.publish(self.motorPos[1])
        else:
            print "Tilt Center Position"

    def lowPassFilter(self, speed):
        pass

    def createWindows(self, imageName, imageToShow, WindowSize = (640,320)):
        cv2.namedWindow(imageName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(imageName, WindowSize)
        cv2.imshow(imageName, imageToShow)
