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
from std_msgs.msg import UInt16 , Int64


VERBOSE = True
DEBUG = True

class ProcessImageBasedColor:
    def __init__(self):
        # Parameters to detect normal red instead of the red of a rasp (testing):
        # self.hul = 109
        # self.huh = 203
        # self.sal = 100
        # self.sah = 255
        # self.val = 11
        # self.vah = 106

        self.hul = 101
        self.huh = 140
        self.sal = 051
        self.sah = 255
        self.val = 0
        self.vah = 40

        self.minimumAreaOfObject = 2000
        self.maximumAreaOfObject = 7000

    def createTrackBarWindows(self):
        """
            Function to set the window that change all the HSV parameters.
        """
        def nothing(x):
            pass

        #assign strings for ease of coding
        cv2.namedWindow('Colorbars')
        hh = 'Hue High'
        hl = 'Hue Low'
        sh = 'Sat High'
        sl = 'Sat Low'
        vh = 'Value High'
        vl = 'Value Low'
        wnd = 'Colorbars'


        #Begin Creating trackbars for each
        cv2.createTrackbar(hl, wnd, self.hul, 255, nothing)
        cv2.createTrackbar(hh, wnd, self.huh, 255, nothing)
        cv2.createTrackbar(sl, wnd, self.sal, 255, nothing)
        cv2.createTrackbar(sh, wnd, self.sah, 255, nothing)
        cv2.createTrackbar(vl, wnd, self.val, 255, nothing)
        cv2.createTrackbar(vh, wnd, self.vah, 255, nothing)

        #read trackbar positions for each trackbar
        self.hul = cv2.getTrackbarPos(hl, wnd)
        self.huh = cv2.getTrackbarPos(hh, wnd)
        self.sal = cv2.getTrackbarPos(sl, wnd)
        self.sah = cv2.getTrackbarPos(sh, wnd)
        self.val = cv2.getTrackbarPos(vl, wnd)
        self.vah = cv2.getTrackbarPos(vh, wnd)

    def create_mask(self,raw_img):
        """
            Method to create a mask for detect raspberrys with a raw img.
            Returns:
                self.mask value changed to store the mask img.
        """
        # Threshold the image
        img_yuv = cv2.cvtColor(raw_img, cv2.COLOR_BGR2YUV)
        # equalize the histogram of the Y channel
        img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
        # # convert the YUV image back to RGB format
        img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        ##----- Light correction ----------------------------------------------------
        lab = cv2.cvtColor(raw_img, cv2.COLOR_BGR2LAB)
        #-----Splitting the LAB image to different channels-------------------------
        l, a, b = cv2.split(lab)
        #-----Applying CLAHE to L-channel-------------------------------------------
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        # cv2.imshow('CLAHE output', cl)
        #-----Merge the CLAHE enhanced L-channel with the a and b channel-----------
        limg = cv2.merge((cl,a,b))
        # cv2.imshow('limg', limg)
        #-----Converting image from LAB Color model to HSV model--------------------
        light_corrected = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        # cv2.imshow('Light control', light_corrected)
        reds_to_blues = cv2.cvtColor(light_corrected, cv2.COLOR_BGR2RGB )
        # cv2.imshow('Colors changed', reds_to_blues)
        #Convert image to HSV space color
        img_hsv = cv2.cvtColor(reds_to_blues,cv2.COLOR_BGR2HSV)
        #make array for final values
        self.createTrackBarWindows()
        hsv_low=np.array([self.hul, self.sal, self.val])
        hsv_max=np.array([self.huh, self.sah, self.vah])
        # Threshold the image
        thres_img = cv2.inRange(img_hsv, hsv_low, hsv_max)
        kernel = np.ones((5,5), np.uint8)
        proc_img = cv2.erode(thres_img, kernel, iterations=1)
        proc_img = cv2.dilate(proc_img, kernel, iterations=2)
        cv2.namedWindow('process image', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('process image', (940,640))
        cv2.imshow('process image', proc_img)
        cv2.waitKey(3)
        return proc_img

    def find_countor(self,frame):
        mask_image = self.create_mask(frame)
        im2,contours,hierarchy = cv2.findContours(mask_image, 1, 2)
        imgg = frame


        total_centers = []
        cx = 0
        cy = 0
        for i, cnt in enumerate(contours):
            moment = cv2.moments(cnt)
            area = moment['m00']
            if area >= self.minimumAreaOfObject:
                #Draw the contours
                # print (area)
                imgg = cv2.drawContours(imgg, contours, i, (0,255,0), 2)
                #Calculate the center coordinate of each contour
                cx = int(moment['m10']/moment['m00'])
                cy = int(moment['m01']/moment['m00'])
                imgg = cv2.circle(imgg,(cx,cy),5,(255,0,0),-1)
                total_centers.append((cx,cy))


        cv2.namedWindow('countours', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('countours', (940,640))
        cv2.imshow('countours', imgg)
        centerPoint = np.array([cx, cy])
        return centerPoint

class ArucoFinder:

    def __init__(self):
        # self.processImageBasedColor = ProcessImageBasedColor()
        self.TrackingData = [['time', 'speed', 'diff']]
        self.oldTime = 0.0
        self.timeDiff = 0.0
        self.oldNumber = 0
        self.sameNumber = 0
        self.saveDataAble = False
        # Define the publisher to the left motor
        # self.leftMotorPub = rospy.Publisher('/left/pan/move', Vector3, queue_size=1)
        self.image_sub = rospy.Subscriber('/stereo/left/image_color', Image, self.image_callback, queue_size=1) #/stereo/right/image_color
        self.ideaToTrackSub = rospy.Subscriber('/ideaNumber',Int64, self.ideaToTrackCallback, queue_size=1)
        self.ideatToTrack = 1
        self.bridge = CvBridge()
        self.image = 0
        self.image_state = False

        # This virable use to filter the

        self.imageSize = np.array([2048 , 1080])
        lineSize = 20
        self.horozintalLine1 = np.array([self.imageSize[0]/2, self.imageSize[1]/2-lineSize])
        self.horozintalLine2 = np.array([self.imageSize[0]/2, self.imageSize[1]/2+lineSize])
        self.verticalLine1 = np.array([self.imageSize[0]/2-lineSize, self.imageSize[1]/2])
        self.verticalLine2 = np.array([self.imageSize[0]/2+lineSize, self.imageSize[1]/2])
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
        self.exponatialGain = [0.004, 0.0035]
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
        self.saveDateAfterFinish()
        for i in range(50):
            self.motorPos[0].data = 0.0
            self.motorPos[1].data = 0.0
            self.currentPos = [0.0, 0.0]
            self.motorPublisher.publish(self.motorPos[0])
            self.tiltMotorPublisher.publish(self.motorPos[1])
            self.TrackingData = []
            self.oldTime = 0.0
            self.timeDiff = 0.0


    def ideaToTrackCallback(self,data):
        self.ideatToTrack = data.data

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
                centerPoint = self.findAurco(self.image)
                # centerPoint = self.processImageBasedColor.find_countor(self.image)
                differences = self.calculateDifferences(centerPoint)
                print ("diff: i%", differences)
                self.moveMotor(differences[0])
                self.TiltMoveMotor(differences[1])
                ikey = cv2.waitKey(3)
                if ikey == ord('q') : #or (self.sameNumber == 15 and self.saveDataAble == True):
                    self.sameNumber = 0
                    break

    def calculateDifferences(self, centerPoint):
        if centerPoint is not None:
            return  centerPoint - self.imageSize/2

        else:
            return np.array([0 , 0])


    def findAurco(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        parameters =  aruco.DetectorParameters_create()

        #lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        image = aruco.drawDetectedMarkers(image, corners)

        centerPoint = self.returnPointForSelectedIde(corners,ids,self.ideatToTrack)
        if centerPoint is not None:
		          image = cv2.circle(image, (centerPoint[0],centerPoint[1]) , 5, (255,0,255) , -1)
        # image = cv2.circle(image, (self.imageSize[0]/2, self.imageSize[1]/2) , 5, (0,0,255) , -1)
        lineSize = 20
        image = cv2.line(image,(self.imageSize[0]/2, self.imageSize[1]/2-lineSize),(self.imageSize[0]/2, self.imageSize[1]/2+lineSize),(0,0,255),2)
        image = cv2.line(image,(self.imageSize[0]/2-lineSize, self.imageSize[1]/2),(self.imageSize[0]/2+lineSize, self.imageSize[1]/2),(0,0,255),2)
        self.createWindows("Aruco", image,(900,600))
        return centerPoint

    def returnCenterOfArcuo(self, markerCorners,ide = 0):
        centerPoint = np.array([0, 0])
        if len(markerCorners) > 0:
            centerPoint[0] = int((markerCorners[ide][0][1][0] + markerCorners[ide][0][2][0] + markerCorners[ide][0][3][0] + markerCorners[ide][0][0][0])/4)
            centerPoint[1] = int((markerCorners[ide][0][1][1] + markerCorners[ide][0][2][1] + markerCorners[ide][0][3][1] + markerCorners[ide][0][0][1])/4)
        else:
            centerPoint = self.imageSize/2
        return centerPoint

    def returnPointForSelectedIde(self,corners,ids,number):
    	if ids is not None:
    		for x, ide in enumerate(ids,0):
    			if ide == number:
    				centerPoint = self.returnCenterOfArcuo(corners, x)
    				return centerPoint
    	return None

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

        # Save the speed
        if self.saveDataAble == True:
            self.storeData(speed, value)
            self.checkSpeedRepetedlly(value)

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

    def checkSpeedRepetedlly(self, speed):
        if speed == self.oldNumber:
            self.sameNumber += 1
        else:
            self.oldNumber = speed
            self.sameNumber = 0


    def lowPassFilter(self, speed):
        pass

    def createWindows(self, imageName, imageToShow, WindowSize = (640,320)):
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



    def saveData(self):
        import csv
        import os
        for i in range(10):
            name = '/home/abdulla/TrackingSpeed/'+ str("%0.4f" % self.exponatialGain) + '/'+ str("%0.2f" % self.mapExponatialValue)+'/00' + str(i) +'.csv'
            fileBool = os.path.isfile(name)    # False
            if fileBool == False:
                with open(name, "wb") as f:
                    writer = csv.writer(f)
                    writer.writerows(self.TrackingData)
                break

        print "Data Saved"

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
    rospy.init_node('ArucoFinder', anonymous = True)
    arucoFinder = ArucoFinder()
    try:
        arucoFinder.trackObject()

    except KeyboardInterrupt:
        print "Shutting ArucoFinder node down"
    cv2.destroyAllWindows()
    arucoFinder.moveToZero()

def saveData():
        rospy.init_node('ArucoFinder', anonymous = True)
        arucoFinder = ArucoFinder()
        try:
            for i in range(10):
                print "iteration", i
                time.sleep(1)
                arucoFinder.trackObject()
                arucoFinder.saveData()
                arucoFinder.moveToZero()



        except KeyboardInterrupt:
            print "Shutting ArucoFinder node down"
        cv2.destroyAllWindows()
        arucoFinder.saveData()
        arucoFinder.moveToZero()
if __name__ == '__main__':
    main(sys.argv)
    # saveData()
