import rospy
import cv2
import cv2.aruco as aruco
import sys
import numpy as np
import time


class TrackByAruco:

    def __init__(self, imageSize):
        self.imageSize = imageSize
        self.ideatToTrack = 1

    def setIdeaToTrack(self, ideatToTrack):
        self.ideatToTrack = ideatToTrack


    def trackObject(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        parameters =  aruco.DetectorParameters_create()

        #lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        image = aruco.drawDetectedMarkers(image, corners)

        centerPoint = self.returnPointForSelectedIde(corners,ids,self.ideatToTrack)
        if centerPoint is not None:
            image = cv2.circle(image, (centerPoint[0],centerPoint[1]) , 5, (255,0,255) , -1)
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

    def createWindows(self, imageName, imageToShow, WindowSize = (640,320)):
        cv2.namedWindow(imageName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(imageName, WindowSize)
        cv2.imshow(imageName, imageToShow)
        cv2.waitKey(10)
