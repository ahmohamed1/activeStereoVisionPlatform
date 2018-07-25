import rospy
import cv2
import sys
import numpy as np
import time



class ProcessImageBasedColor:
    def __init__(self, imageSize, multiTarget=False):
        # Parameters to detect normal red instead of the red of a rasp (testing):
        self.hul = 27
        self.huh = 134
        self.sal = 45
        self.sah = 255
        self.val = 00
        self.vah = 255

        self.imageSize = imageSize

        # self.hul = 21
        # self.huh = 255
        # self.sal = 92
        # self.sah = 255
        # self.val = 37
        # self.vah = 255

        self.multiTarget = multiTarget
        self.minimumAreaOfObject = 3000
        self.maximumAreaOfObject = 15000

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

    def drawCrossMark(self, imgg, cx, xy):
        lineSize = 20
        targetCenter = np.array([cx, xy]);
        imgg = cv2.line(imgg,(targetCenter[0], targetCenter[1]-lineSize),(targetCenter[0], targetCenter[1]+lineSize), (255, 0, 0), 2)
        imgg = cv2.line(imgg,(targetCenter[0]-lineSize, targetCenter[1]),(targetCenter[0]+lineSize, targetCenter[1]), (255, 0, 0), 2)
        return imgg


    def trackObject(self,frame):
        mask_image = self.create_mask(frame)
        im2,contours,hierarchy = cv2.findContours(mask_image, 1, 2)
        imgg = frame

        total_centers = []
        cx = 0
        cy = 0
        j = 1
        for i, cnt in enumerate(contours):
            moment = cv2.moments(cnt)
            area = moment['m00']
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h = cv2.boundingRect(cnt)
            value = (float)(h) / w

            if area > self.minimumAreaOfObject and area < self.maximumAreaOfObject and (len(approx) > 8): # and (value > 0.60 and value < 2.75):
                #imgg = cv2.drawContours(imgg, contours, i, (0,255,0), 2)
                #Calculate the center coordinate of each contour
                cx = int(moment['m10']/moment['m00'])
                cy = int(moment['m01']/moment['m00'])
                # imgg = cv2.circle(imgg,(cx,cy),5,(255,0,0),-1)
                # imgg = self.drawCrossMark(imgg, cx, cy)
                # font = cv2.FONT_HERSHEY_SIMPLEX
                # imgg = cv2.putText(imgg,str(j),(cx,cy), font, 3,(255,0,255),2,cv2.LINE_AA)
                j = j + 1
                # print ("is circle: ", cv2.arcLength(cnt,True))
                total_centers.append((cx,cy))


        # lineSize = 20
        # imgg = cv2.line(imgg,(self.imageSize[0]/2, self.imageSize[1]/2-lineSize),(self.imageSize[0]/2, self.imageSize[1]/2+lineSize),(0,0,255),2)
        # imgg = cv2.line(imgg,(self.imageSize[0]/2-lineSize, self.imageSize[1]/2),(self.imageSize[0]/2+lineSize, self.imageSize[1]/2),(0,0,255),2)
        cv2.namedWindow('countours', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('countours', (940,640))
        cv2.imshow('countours', imgg)
        if self.multiTarget:
            return total_centers
        else:
            return np.array([cx, cy])
