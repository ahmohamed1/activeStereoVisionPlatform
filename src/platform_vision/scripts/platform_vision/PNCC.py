#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
import math
from skimage.measure import compare_ssim

VERBOSE = True
DEBUG = True

class FastMatchingPyramid:
    def __init__(self, imageSize, pyramidLevel = 7, windowSize = 80, grayImage = False, showImage = True, drawDifferencesInImage=False, operatingName='motor controller'):
        self.pyramidLevel = pyramidLevel
        self.imageSize = imageSize
        self.templateSize = windowSize
        self.templateSize2D = [windowSize,windowSize]
        self.x1 = (self.imageSize[0]/2 - (self.templateSize2D[0]/2))
        self.x2 = (self.imageSize[0]/2 + (self.templateSize2D[0]/2))
        self.y1 = (self.imageSize[1]/2 - (self.templateSize2D[0]/2))
        self.y2 = (self.imageSize[1]/2 + (self.templateSize2D[0]/2))
        self.showImage = showImage
        self.grayImage = grayImage
        self.windowname = operatingName
        self.templateName = operatingName + 'template'
        self.template = None
        self.drawDifferencesInImage = drawDifferencesInImage
        if self.drawDifferencesInImage:
            self.matchingLine = None

        if self.windowname != 'Slave ':
            # cv2.namedWindow(self.windowname, cv2.WINDOW_NORMAL)
            self.terminateButton = False

    def my_mouse_callback(self, event,x,y,flags,param):
        # if event==cv2.EVENT_LBUTTONDOWN:
        #     self.saveImage(self.fastMatchingPyramid.getTemplate())
        #     pass
        if event==cv2.EVENT_RBUTTONDOWN:
            self.terminateButton = True

    def getTerminatedState(self):
        return self.terminateButton

    def getTemplateSize(self):
        return self.templateSize

    def setTemplateSize(self,size):
        self.templateSize = size

    def setTemplateSize2D(self,size):
        self.templateSize2D = size


    def setgrayImage(grayImageState):
        self.grayImage = grayImageState

    def saveImage(self, templateImage):
        self.savenumber += 1
        tempImgStr = '/home/abdulla/dev/Data/' + str(self.savenumber) + 'template.jpg'
        leftImgStr = '/home/abdulla/dev/Data/' + str(self.savenumber) + 'left.jpg'
        rightImgStr = '/home/abdulla/dev/Data/' + str(self.savenumber) + 'right.jpg'
        cv2.imwrite(tempImgStr, templateImage)
        cv2.imwrite(rightImgStr, self.right_image)
        cv2.imwrite(leftImgStr, self.left_image)
        print ('Image saved')


    def createTemplate(self, leftImage, coordinate):
        if coordinate[0] - (self.templateSize2D[0]/2) < 0 :
            self.x1 = 0
            self.x2 = self.templateSize2D[0]
        elif coordinate[0] + (self.templateSize2D[0]/2) > self.imageSize[0]:
            self.x1 = int(self.imageSize[0] - self.templateSize2D[0])
            self.x2 = self.imageSize[0]
        else:
            self.x1 = int(coordinate[0] - (self.templateSize2D[0]/2))
            self.x2 = int(coordinate[0] + (self.templateSize2D[0]/2))

        if coordinate[1] - (self.templateSize2D[1]/2) < 0 :
            self.y1 = 0
            self.y2 = self.templateSize2D[1]
        elif coordinate[1] + (self.templateSize2D[1]/2) > self.imageSize[1]:
            self.y1 = int(self.imageSize[1] - self.templateSize2D[1])
            self.y2 = self.imageSize[1]
        else:
            self.y1 = int(coordinate[1] - (self.templateSize2D[1]/2))
            self.y2 = int(coordinate[1] + (self.templateSize2D[1]/2))
        # print (self.x1, self.x2, self.y1, self.y2)
        self.template = leftImage[self.y1:self.y2, self.x1:self.x2]
        # cv2.imshow(self.templateName, self.template)
        # cv2.waitKey(1)

    def createMultipleTemplate(self, leftImage, centerList):
        templateList = []
        for coordinate in centerList:
            if coordinate[0] - (self.templateSize/2) < 0 :
                x1 = 0
                x2 = self.templateSize
            elif coordinate[0] + (self.templateSize/2) > self.imageSize[0]:
                x1 = self.imageSize[0] - self.templateSize
                x2 = self.imageSize[0]
            else:
                x1 = coordinate[0] - (self.templateSize/2)
                x2 = coordinate[0] + (self.templateSize/2)

            if coordinate[1] - (self.templateSize/2) < 0 :
                y1 = 0
                y2 = self.templateSize
            elif coordinate[1] + (self.templateSize/2) > self.imageSize[1]:
                y1 = self.imageSize[1] - self.templateSize
                y2 = self.imageSize[1]
            else:
                y1 = coordinate[1] - (self.templateSize/2)
                y2 = coordinate[1] + (self.templateSize/2)

            print (x1, x2, y1, y2)
            template = leftImage[y1:y2, x1:x2]
            templateList.append(template)
        return templateList

    def compareTemplate(self, image1, image2):
        s = compare_ssim(image1, image2,multichannel=True)
        return s

    def setTemplate(self, template):
        self.template = template

    def getTemplate(self):
        return self.template


    def trackObject(self, rightImage):
        if self.template is not None:
            return self.fastTemplateMatch(rightImage, self.template, self.pyramidLevel)
        else:
            return np.array([self.imageSize[0]/2, self.imageSize[1]/2])

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
        # if self.windowname != 'Slave ':
            # cv2.setMouseCallback(self.windowname, self.my_mouse_callback)
        results = []

        if src_refimg.shape[2] == 1:
            ## Change BGR to Grayscale
            gray_refimg = cv2.cvtColor(src_refimg, cv2.COLOR_BGR2GRAY)
            gray_tplimg = cv2.cvtColor(src_tplimg, cv2.COLOR_BGR2GRAY)
            ## Build image pyramid
            refimgs = self.buildPyramid(gray_refimg, maxleval)
            tplimgs = self.buildPyramid(gray_tplimg, maxleval)
        else:
            refimgs = self.buildPyramid(src_refimg, maxleval)
            tplimgs = self.buildPyramid(src_tplimg, maxleval)


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

            T, threshed = cv2.threshold(result, 0.79, 1., cv2.THRESH_TOZERO)
            results.append(threshed)
        # self.createWindows("results", threshed, (900,600))
        if self.drawDifferencesInImage:
            rows = result.shape[0]
            self.matchingLine = result[rows/2, :]
            windowsizeExtra = np.full((self.templateSize/2),0.8)
            self.matchingLine = np.hstack([windowsizeExtra, self.matchingLine])
        # print (self.matchingLine)
        return threshed
        #return results


    def fastTemplateMatch(self, refimg, tplimg, maxleval = 5):
        """Fast template match.
        """
        ## Call fastTemplateMatchInPyramid()
        result = self.fastTemplateMatchPyramid(refimg, tplimg, maxleval)

        ## Analysis the result
        minval, maxval, minloc, maxloc = cv2.minMaxLoc(result)
        if maxval > 0.8:
            pt1 = maxloc
            pt2 = (maxloc[0] + tplimg.shape[1], maxloc[1] + tplimg.shape[0])
            centerPoint = (maxloc[0] + tplimg.shape[1]/2, maxloc[1] + tplimg.shape[0]/2)
            # print("Found the template region: {} => {}".format(pt1,pt2))
            dst = refimg.copy()
            cv2.rectangle(dst, pt1, pt2, (0,255,0), 2)
            if self.showImage:
                if self.drawDifferencesInImage:
                    self.Xx = np.arange(self.matchingLine.size)
                    pts = np.vstack((self.Xx,self.matchingLine*self.imageSize[1])).astype(np.int32).T
                    dst = cv2.polylines(dst, [pts], isClosed=False, color=(255,255,255), thickness=2)
                lineSize = 20
                imageToShow = cv2.line(dst,(self.imageSize[0]/2, self.imageSize[1]/2-lineSize),(self.imageSize[0]/2, self.imageSize[1]/2+lineSize),(0,0,255),2)
                imageToShow = cv2.line(dst,(self.imageSize[0]/2-lineSize, self.imageSize[1]/2),(self.imageSize[0]/2+lineSize, self.imageSize[1]/2),(0,0,255),2)
                x_offset=y_offset=10
                s_img = self.template
                dst = cv2.rectangle(dst,(0,0),(s_img.shape[0]+y_offset,s_img.shape[0]+y_offset),(0,0,255),-1)
                dst[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1]] = s_img
                if self.windowname != 'Slave':
                    # self.createWindows(self.windowname, dst, (900,600))
                    pass
            if self.windowname != 'Slave ':
                return centerPoint
            else :
                return dst, centerPoint
        else:
            print("Cannot find the template in the origin image!")

    def createWindows(self, imageName, imageToShow, WindowSize = (900,600)):
        lineSize = 20
        imageToShow = cv2.line(imageToShow,(self.imageSize[0]/2, self.imageSize[1]/2-lineSize),(self.imageSize[0]/2, self.imageSize[1]/2+lineSize),(0,0,255),2)
        imageToShow = cv2.line(imageToShow,(self.imageSize[0]/2-lineSize, self.imageSize[1]/2),(self.imageSize[0]/2+lineSize, self.imageSize[1]/2),(0,0,255),2)
        # cv2.namedWindow(imageName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(imageName, WindowSize)
        cv2.imshow(imageName, imageToShow)
        ikey = cv2.waitKey(1)
