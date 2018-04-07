#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

from pid_controller import PIDController

from dynamic_reconfigure.server import Server
from platform_vision.cfg import pidConfig

VERBOSE = True
DEBUG = True

class FastMatchingPyramid:

    def __init__(self):
        # Define the publisher to the left motor
        # self.leftMotorPub = rospy.Publisher('/right/pan/move', Vector3, queue_size=2)
        self.leftMotorPub = rospy.Publisher('/right/pan/move', Float64, queue_size=2)
        self.left_image_sub = rospy.Subscriber('/stereo/left/image_color', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/stereo/right/image_color', Image, self.right_image_callback)
        self.bridge = CvBridge()

        self.left_image = None
        self.left_image_state = False
        self.right_image = None
        self.right_image_state = False
        self.savenumber = 0

        self.pyramidLevel = 10
        self.imageSize = np.array([2048 , 1080])
        self.templateSize = 80
        self.x1 = (self.imageSize[0]/2 - (self.templateSize/2))
        self.x2 = (self.imageSize[0]/2 + (self.templateSize/2))
        self.y1 = (self.imageSize[1]/2 - (self.templateSize/2))
        self.y2 = (self.imageSize[1]/2 + (self.templateSize/2))

        self.exponatialGain = [0.003, 0.0035]
        self.mapExponatialValue = [0.2, 0.35]
        self.motorMinLimit = -120
        self.motorMaxLimit = 120
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
        leftImgStr = str(self.savenumber) + 'template.jpg'
        rightImgStr = str(self.savenumber) + 'right.jpg'
        cv2.imwrite(leftImgStr, templateImage)
        cv2.imwrite(rightImgStr, self.right_image)
        print ('Image saved')


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
        if abs(value) > 80 :
            self.currentPos[0] += speed
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            if self.currentPos[0] < self.motorMaxLimit and self.currentPos[0] > self.motorMinLimit :
                self.leftMotorPub.publish(self.motorPos[0])
        elif abs(value) <  80 and abs(value) >  20:
            self.currentPos[0] -= value * 0.0025
            self.motorPos[0].data = self.currentPos[0]
            # print("Motor speed: ", self.currentPos)
            self.leftMotorPub.publish(self.motorPos[0])

        else:
            print "Pan Center Position"


    def convertROSToCV(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            return cv_image, True
        except CvBridgeError, e:
            print e

    def left_image_callback(self, data):
        self.left_image, self.left_image_state = self.convertROSToCV(data)

    def right_image_callback(self, data):
        self.right_image, self.right_image_state = self.convertROSToCV(data)

    def showImage(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            if self.left_image_state is True and self.right_image_state is True :
                ## call the function
                # right_image_with_rectangle = cv2.rectangle(self.right_image, (self.x1, self.y1) , (self.x2, self.y2), (0,0,255), 3)
                template = self.left_image[self.y1:self.y2, self.x1:self.x2]
                cv2.imshow("template image", template)
                # self.createWindows("left image", self.left_image)
                # self.createWindows("right image", right_image_with_rectangle)
                self.fastTemplateMatch(self.right_image, template, 7)
                ikey = cv2.waitKey(3)
                if ikey == ord('q'):
                    self.moveToZero()
                    exit()
                elif ikey == ord('s'):
                    self.saveImage(template)

    def trackObject(self):
        rate = rospy.Rate(60) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            if self.left_image_state is True and self.right_image_state is True :
                ## call the function
                # right_image_with_rectangle = cv2.rectangle(self.right_image, (self.x1, self.y1) , (self.x2, self.y2), (0,0,255), 3)
                template = self.left_image[self.y1:self.y2, self.x1:self.x2]
                cv2.imshow("template image", template)
                # self.createWindows("left image", self.left_image)
                centerPoint = self.fastTemplateMatch(self.right_image, template, self.pyramidLevel)
                differences = self.calculateDifferences(centerPoint)
                self.moveMotor(differences[0])
                # timestamp = rospy.get_time()
                # speed = self.PidController.update(differences[0], timestamp)
                # self.PublishMotorAtSpeedRate(speed)
                ikey = cv2.waitKey(3)
                if ikey == ord('q'):
                    self.moveToZero()
                    exit()

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


    def fastTemplateMatchPyramid(self, src_refimg, src_tplimg, maxleval):
        """Do fast template matching using matchTemplate plus an approximation
        through pyramid construction to improve it's performance on large images.
        """
        results = []

        ## Change BGR to Grayscale
        gray_refimg = cv2.cvtColor(src_refimg, cv2.COLOR_BGR2GRAY)
        gray_tplimg = cv2.cvtColor(src_tplimg, cv2.COLOR_BGR2GRAY)

        ## Build image pyramid
        refimgs = self.buildPyramid(gray_refimg, maxleval)
        tplimgs = self.buildPyramid(gray_tplimg, maxleval)

        ## Do template match
        for idx in range(0, maxleval+1):
            refimg = refimgs[idx]
            tplimg = tplimgs[idx]

            # On the first level performs regular template matching.
            # On every other level, perform pyramid transformation and template matching
            # on the predefined ROI areas, obtained using the result of the previous level.
            # Uses contours to define the region of interest and perform TM on the areas.
            if idx == 0:
                result = cv2.matchTemplate(refimg, tplimg, cv2.TM_CCORR_NORMED)
            else:
                mask = cv2.pyrUp(threshed)
                mask8u = cv2.inRange(mask, 0, 255)
                _,contours,_ = cv2.findContours(mask8u, cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_NONE)

                tH, tW = tplimg.shape[:2]
                for cnt in contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    src = refimg[y:y+h+tH, x:x+w+tW]
                    result = cv2.matchTemplate(src, tplimg, cv2.TM_CCORR_NORMED)

            T, threshed = cv2.threshold(result, 0.90, 1., cv2.THRESH_TOZERO)
            results.append(threshed)

        return threshed
        #return results


    def fastTemplateMatch(self, refimg, tplimg, maxleval = 5):
        """Fast template match.
        """
        ## Call fastTemplateMatchInPyramid()
        result = self.fastTemplateMatchPyramid(refimg, tplimg, maxleval)

        ## Analysis the result
        minval, maxval, minloc, maxloc = cv2.minMaxLoc(result)
        if maxval > 0.9:
            pt1 = maxloc
            pt2 = (maxloc[0] + tplimg.shape[1], maxloc[1] + tplimg.shape[0])
            centerPoint = (maxloc[0] + tplimg.shape[1]/2, maxloc[1] + tplimg.shape[0]/2)
            # print("Found the template region: {} => {}".format(pt1,pt2))
            dst = refimg.copy()
            cv2.rectangle(dst, pt1, pt2, (0,255,0), 2)
            self.createWindows("Result", dst, (900,600))
            return centerPoint
        else:
            print("Cannot find the template in the origin image!")

    def createWindows(self, imageName, imageToShow, WindowSize = (900,600)):
        cv2.namedWindow(imageName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(imageName, WindowSize)
        cv2.imshow(imageName, imageToShow)

def main(args):
    rospy.init_node('FastMatchingPyramid', anonymous = True)
    fastMatchingPyramid = FastMatchingPyramid()
    try:
        fastMatchingPyramid.trackObject()
        # rospy.spin()

    except KeyboardInterrupt:
        print "Shutting FastMatchingPyramid node down"
    cv2.destroyAllWindows()
    fastMatchingPyramid.moveToZero()

if __name__ == '__main__':
    main(sys.argv)
