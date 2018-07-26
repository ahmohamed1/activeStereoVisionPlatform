# import the necessary packages
from scipy import ndimage
import numpy as np
import cv2

import os.path


def cropTarget(colorImage, x, y ,r, ratio_width, ratio_hieght, extraRaduis = 20):
	if x != 0 and y !=0:
		X = x * ratio_width
		Y = y * ratio_hieght
		R = int(r * (np.sqrt(X*X + Y*Y)/np.sqrt(x*x + y*y)))
		# R =r
		# print ('X, T, R', X , '->', x,Y, '->',y,R, '->',r)
		cropedImageSizeX1 = int(X - (R + extraRaduis))
		cropedImageSizeX2 = int(X + (R + extraRaduis))
		cropedImageSizeY1 = int(Y - (R + extraRaduis))
		cropedImageSizeY2 = int(Y + (R + extraRaduis))
		h = colorImage.shape[0]
		w = colorImage.shape[1]
		if cropedImageSizeX1 < 0:
			cropedImageSizeX1 = 0
			cropedImageSizeX2 = R
			# print ("X1,X2", x1,x2)
		if cropedImageSizeX2 > w:
			cropedImageSizeX1 = w - R
			cropedImageSizeX2 = w
		if cropedImageSizeY1 < 0:
			cropedImageSizeY1 = 0
			cropedImageSizeY2 = R
			# print ("y1,y2", y1,y2)
		if cropedImageSizeY2 > h:
			cropedImageSizeY1 = h - R
			cropedImageSizeY2 = h
		cropedTarget = colorImage[cropedImageSizeY1:cropedImageSizeY2, cropedImageSizeX1:cropedImageSizeX2]
		return cropedTarget
	else :
		return None

def compute_avarge_around_most_interest(img, point, window_size=15):
    w, h = img.shape
    x1 = point[0]-window_size
    x2 = point[0]+window_size
    y1 = point[1]-window_size
    y2 = point[1]+window_size
    if x1 < 0:
        x1 = 0
        x2 = window_size * 2
        # print ("X1,X2", x1,x2)
    if x2 > w:
        x1 = w - window_size * 2
        x2 = w
    if y1 < 0:
        y1 = 0
        y2 = window_size*2
        # print ("y1,y2", y1,y2)
    if y2 > h:
        y1 = h - window_size*2
        y2 = h

    rectangule = img[x1:x2, y1:y2]
    mean = rectangule.mean()
    return mean


def compute_newCoordinate(img, point, window_size=15):
	h, w = img.shape
	mask = np.zeros((h,w),np.uint8)
	X = point[0]
	Y = point[1]
	R = window_size
	cropedImageSizeX1 = int(X - R)
	cropedImageSizeX2 = int(X + R)
	cropedImageSizeY1 = int(Y - R)
	cropedImageSizeY2 = int(Y + R)

	if cropedImageSizeX1 < 0:
		cropedImageSizeX1 = 0
		cropedImageSizeX2 = R
		# print ("X1,X2", x1,x2)
	if cropedImageSizeX2 > w:
		cropedImageSizeX1 = w - R
		cropedImageSizeX2 = w
	if cropedImageSizeY1 < 0:
		cropedImageSizeY1 = 0
		cropedImageSizeY2 = R
		# print ("y1,y2", y1,y2)
	if cropedImageSizeY2 > h:
		cropedImageSizeY1 = h - R
		cropedImageSizeY2 = h
	mask=cv2.rectangle(mask, (cropedImageSizeX1,cropedImageSizeY1), (cropedImageSizeX2,cropedImageSizeY2), 255, -1)
	# cv2.imshow('mask', mask)
	return mask


def findContoursRegion(image):
	cnts = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	cv2.drawContours(image, cnts,-1 , (150), -1)
	cv2.imshow('imageCountor', image)

def returnThresholdMask(img, center, windowSize= 20):
	x1 = 0
	y1 = 0
	x2 = 620
	y2 = 420
	image = None
	image = img.copy()
	image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
	# cv2.rectangle(image,(x1,y1),(x2,y2), 255, -1 )
	# cv2.imshow('grabCut image', image)
	# cv2.waitKey(0)
	rect = (x1,y1,x2,y2)
	mask = np.zeros(image.shape[:2],np.uint8)
	bgdModel = np.zeros((1,65),np.float64)
	fgdModel = np.zeros((1,65),np.float64)
	cv2.grabCut(image,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
	mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
	image = image*mask2[:,:,np.newaxis]
	cv2.imshow('grabCut image', image)
	cv2.waitKey(0)

def rotateImage(image, angle):
	rows = image.shape[0]
	cols = image.shape[1]

	M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
	dst = cv2.warpAffine(image,M,(cols,rows))
	return dst

def openCVWaterShed(img, colorImage):
	gray = img.copy()
	ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	# cv2.imshow('thresh', thresh)
	# cv2.waitKey(0)
	# noise removal
	kernel = np.ones((3,3),np.uint8)
	opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
	# sure background area
	sure_bg = cv2.dilate(opening,kernel,iterations=3)
	# Finding sure foreground area
	dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
	ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
	# Finding unknown region
	sure_fg = np.uint8(sure_fg)
	unknown = cv2.subtract(sure_bg,sure_fg)
	# Marker labelling
	ret, markers = cv2.connectedComponents(sure_fg)
	# Add one to all labels so that sure background is not 0, but 1
	markers = markers+1
	# Now, mark the region of unknown with zero
	markers[unknown==255] = 0
	markers = cv2.watershed(colorImage,markers)
	colorImage[markers == 2] = [255,255,255]
	# colorImage[markers == -1] = [255,255,255]
	ret, m2 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
	# cv2.imshow('m2', m2)
	_, contours, hierarchy = cv2.findContours(m2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
	for c in contours:
		area = cv2.contourArea(c)
		# if area > 2000:
	cv2.drawContours(colorImage, contours, -1, (0, 255, 0), -1)
	#cv2.imshow('markers1', markers1)
	# cv2.waitKey(0)
	# cv2.imshow('openCVWaterShed', colorImage)
	# cv2.waitKey(0)

def find_countor(saliency,img):
	frame = np.zeros(saliency.shape, saliency.dtype)
	frame = saliency.copy()
	# frame[frame < 120] = 0
	# frame[frame > 120] = 255
	# newImage = cv2.bitwise_and(img, img,mask=frame)
	# cv2.imshow('newImage', newImage)
	imgg = np.zeros(img.shape, img.dtype)
	imgg = img.copy()

	im2,contours,hierarchy = cv2.findContours(frame, 1, 2)
	total_centers = []
	cx = 0
	cy = 0
	for i, cnt in enumerate(contours):
		approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
		moment = cv2.moments(cnt)
		area = moment['m00']
		# if area >= 100 and area <= 1000:
		imgg = cv2.drawContours(imgg, contours, i, (0,255,0), -1)
		#Calculate the center coordinate of each contour
		if moment['m00'] == 0:
			moment['m00'] = 0.001
		cx = int(moment['m10']/moment['m00'])
		cy = int(moment['m01']/moment['m00'])
		imgg = cv2.circle(imgg,(cx,cy),5,(255,0,0),-1)
		if len(approx)==8:
			area = cv2.contourArea(cnt)
			(_cx, _cy), radius = cv2.minEnclosingCircle(cnt)
			imgg = cv2.circle(imgg,(cx,cy), int(radius),(255,0,0),1)
	# cv2.imshow('countours', imgg)
