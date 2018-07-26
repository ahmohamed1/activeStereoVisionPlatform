# import the necessary packages
from skimage.feature import peak_local_max
from skimage.morphology import watershed, disk
from scipy import ndimage
import numpy as np
import cv2

from helpFunctions import *

non_tomato_number = 0

def waterShedSelectedRigon(img, center, minRadius = 5,maxRadius = 60):
	X = 0
	Y = 0
	R = 0
	w, h = img.shape
	imgProcess = None
	imgProcess = np.zeros((w,h),np.uint8)
	mask = compute_newCoordinate(imgProcess, center, 100)
	imgProcess = cv2.bitwise_and(img, img, mask=mask)
	imgToSHow = np.zeros((w, h, 3))
	imgProcess = cv2.blur(imgProcess,(9,9))
	# returnThresholdMask(imgProcess, center, 40)
	# cv2.imshow('imgProcess', imgProcess)
	# cv2.waitKey(0)
	pixelValue = imgProcess[center[1], center[0]]
	raio = pixelValue - pixelValue*0.3
	# thresh = cv2.threshold(imgProcess, raio, pixelValue, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
	thresh = imgProcess.copy()
	thresh[thresh < raio] = 0
	thresh[thresh > raio] = 255
	cv2.imshow('thresh', thresh)
	# cv2.waitKey(0)
	D = ndimage.distance_transform_edt(thresh, indices=False, sampling=[3,3])
	localMax = peak_local_max(D, indices=False, min_distance=minRadius, labels=thresh)

	# perform a connected component analysis on the local peaks,
	# using 8-connectivity, then appy the Watershed algorithm
	markers = ndimage.label(localMax, structure=np.ones((3, 3)))[0]
	labels = watershed(-D, markers, mask=thresh)
	# print("[INFO] {} unique segments found".format(len(np.unique(labels)) - 1))
	colorImage = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
	# loop over the unique labels returned by the Watershed
	# algorithm
	eroroSizeToCenter = 9999999
	cnts = None
	for label in np.unique(labels):
		# if the label is zero, we are examining the 'background'
		# so simply ignore it
		if label == 0:
			continue
		# otherwise, allocate memory for the label region and draw
		# it on the mask
		mask = np.zeros(img.shape, dtype="uint8")
		mask[labels == label] = 255

		# detect contours in the mask and grab the largest one
		_, cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #[-2]
		# print('contour length: ', len(cnts))
		eroroSizeToCenter = 9999999
		for c in cnts:
			color = np.random.randint(0,255,(3)).tolist()
			cv2.drawContours(imgToSHow, cnts,-1 , color, -1)
			# draw a circle enclosing the object
			((x, y), r) = cv2.minEnclosingCircle(c)
			center = np.sqrt(np.power(x-(w/2),2) + np.power(y-(h/2),2))
			# area = cv2.contourArea(c)
			# print ('X ,Y, center = ',  x, y , center)
			if center < eroroSizeToCenter:
				# cv2.putText(imgToSHow, "#{}".format(label), (int(x) - 10, int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
				((X, Y), R) = cv2.minEnclosingCircle(c)
				eroroSizeToCenter = center
				contourReturn = c

	# print ('to center= ', eroroSizeToCenter)
	if cnts != None:
		color = np.random.randint(0,255,(3)).tolist()
		cv2.drawContours(img, cnts,-1 , (0), -1)
		cv2.drawContours(img, cnts,-1 , (0), 12)
	# cv2.circle(imgToSHow, (int(x), int(y)), int(r), color, -1)
	cv2.imshow('colorImage', imgToSHow)
	# cv2.imshow('img', img)
	cv2.waitKey(1)
	return img, cnts, X, Y, R


def trackingSaliencyRegionBase(imgRgb,saliencyImage, minimumPropapilityTHreshold, model, loopindex = 0):
	global non_tomato_number
	alpha = 0.40
	(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(saliencyImage)
	frame = imgRgb.copy()
	_, cnt, x,y,r = waterShedSelectedRigon(saliencyImage, maxLoc,  minRadius = 5,maxRadius = 60)
	x = int(x)
	y = int(y)
	(B, G, R) = imgRgb[y, x]
	X = [[B, G, R]]
	probabilitOfTomato = model.predict(X)
	if x != 0 and y != 0 and r !=0:
		if probabilitOfTomato > 0.5:
			if maxVal/255 > minimumPropapilityTHreshold:
				# (B, G, R) = imgRgb[y, x]
				# X = [[B, G, R]]
				# probabilitOfTomato = model.predict(X)
				# print ('probabilitOfTomato: ', probabilitOfTomato)
				color = np.random.randint(100,255,(3)).tolist()  # Select a random color
				cv2.drawContours(frame, cnt, -1, color, -1)
				cv2.drawContours(frame, cnt, -1, color, 12)
				cv2.addWeighted(frame, alpha, imgRgb, 1 - alpha, 0, imgRgb)
				cv2.putText(imgRgb ,str(round(probabilitOfTomato[0],2)),(x,y+5), 1, 1,(0,255,255),1,cv2.LINE_AA)
				# cv2.putText(imgRgb, "#{}".format(loopindex), (int(x) - 10, int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
				cv2.circle(imgRgb, (x,y), 5, (0, 255, 0),-1)

				# cover the detected region in salinecy matplotlib
				cv2.drawContours(saliencyImage, cnt, -1, 0, -1)
				cv2.drawContours(saliencyImage, cnt, -1, 0, 25)
				cv2.imshow("input",  imgRgb)
				cv2.imshow("saliencyImage",  saliencyImage)
				return  maxVal/255, saliencyImage, [x,y,r], probabilitOfTomato

			else:
				print('All object found')
				return 0, saliencyImage, [0,0,0], probabilitOfTomato
		else:
			print ('Non-tomato : ', non_tomato_number)
			non_tomato_number +=1
			cv2.drawContours(saliencyImage, cnt, -1, 0, -1)
			cv2.drawContours(saliencyImage, cnt, -1, 0, 25)
			return  maxVal/255, saliencyImage, [0,0,0], probabilitOfTomato
	else:
		print('unknow target')
		cv2.circle(saliencyImage, maxLoc, 10, 0 ,-1)
		return  maxVal/255, saliencyImage, [0,0,0], probabilitOfTomato

def cropedRegionOfInterest(imageFullSize, targetPose, ratio_width, ratio_hieght, targetListInOneScene, extraRaduis = 0,  saveTemplate = False):
	cropedImage = cropTarget(imageFullSize,targetPose[0],targetPose[1],targetPose[2], ratio_width, ratio_hieght, extraRaduis = extraRaduis)
	if cropedImage is not None:
		 if cropedImage.shape[0] != 0 and cropedImage.shape[1] != 0:
			 targetListInOneScene.append(cropedImage)
			 cv2.imshow('cropedTarget', cropedImage)
			 if saveTemplate:
				 directoryName = 'outputData/' + str(savedImageIndex) + '.png'
				 fileExist = os.path.isfile(directoryName)
				 while fileExist:
					 savedImageIndex += 1
					 directoryName = 'outputData/' + str(savedImageIndex) + '.png'
					 # path_ = Path(fileName)
					 fileExist = os.path.isfile(directoryName)
				 cv2.imwrite(directoryName,cropedImage)
