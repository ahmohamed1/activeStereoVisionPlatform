#-------------------------------------------------------------------------------
# Name:        main
# Purpose:     Testing the package pySaliencyMap
#
# Author:      Akisato Kimura <akisato@ieee.org>
#
# Created:     May 4, 2014
# Copyright:   (c) Akisato Kimura 2014-
# Licence:     All rights reserved
#-------------------------------------------------------------------------------

import cv2
import matplotlib.pyplot as plt
import pySaliencyMap

import numpy as np
from scipy import spatial

import matplotlib.pyplot as plt
import numpy as np

from skimage.filters.rank import entropy
from skimage.morphology import disk

# import the necessary packages
from skimage.feature import peak_local_max
from skimage.morphology import watershed
from scipy import ndimage

from sklearn.naive_bayes import MultinomialNB
import pickle

from helpFunctions import *
import time

# construct the argument parse and parse the arguments
import argparse
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--radius", type = int, default=11,
	help = "radius of Gaussian blur; must be odd")
ap.add_argument("-i","--image", default='flea3/120_Image.png',
    help= "image required to be process, Include the type of the image( e.g img.png, img.jpg)")

ap.add_argument("-l","--loop", type=int, default= 0,
    help= "image required to be process, Include the type of the image( e.g img.png, img.jpg)")
args = vars(ap.parse_args())

GaussianBlurRadius = args["radius"]
imageName = args['image']
LoopThroughAllImages = args['loop']
FindTime = True
saveTemplate = False
filenameModel = 'outputData\linear_regressor.pkl' #'GaussianNaiveBayes.sav'
non_tomato_number = 0

def trackingSaliency(imgRgb,saliencyImage, listOfObjects, minimumPropapilityTHreshold, model, loopindex = 0):
	alpha = 0.40
	(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(saliencyImage)

	frame = imgRgb.copy()
	cv2.waitKey(0)
	if len(listOfObjects) > 0 and maxVal/255 > minimumPropapilityTHreshold:
		poseA = [maxLoc[0],maxLoc[1]]
		listOfObjectsPose = [i[2] for i in listOfObjects]
		distance,index = spatial.KDTree(np.array(listOfObjectsPose)).query(poseA)
		ellipse = listOfObjects.pop(index)
		# if distance < 10:
		[x,y] = ellipse[2]
		r =  ellipse[3]
		cnt = ellipse[4]
		#predict the probability of tomatos
		(B, G, R) = imgRgb[maxLoc[1],maxLoc[0]]
		X = [[B, G, R]]
		probabilitOfTomato = model.predict(X)
		cv2.circle(imgRgb, maxLoc, 5, (255, 0, 0),-1)

		if probabilitOfTomato > 0.4:
			# print ('probabilitOfTomato = ', probabilitOfTomato[0])
			# print( '(B, G, R)', B, G, R)
			# print('P(r)= ', round((R)/255, 2),' P(g)= ', round(G/255, 2), 'P(tomato)= ', round((R)/255 * ((G/255)), 2), 'raio=' ,round(G/R, 2)  )
			# print ('Distance: ', distance, " Indec: ",index, 'Pose: ',listOfObjectsPose[index], "Actual Pose: ", poseA  )
			# print ('coordinate: ', x,y,r)
			color = np.random.randint(100,255,(3)).tolist()  # Select a random color
			# cv2.circle(saliencyImage, maxLoc, int(r), (0,0,0), -1)
			# cv2.circle(saliencyImage, maxLoc, int(r), (0,0,0), 10)
			cv2.drawContours(saliencyImage, cnt, -1, (0,0,0), -1)
			cv2.drawContours(saliencyImage, cnt, -1, (0,0,0), 15)
			# cv2.ellipse(frame, ellipse[1], color, -1,cv2.LINE_AA)
			# cv2.ellipse(frame, ellipse[1], color, -1,cv2.LINE_AA)
			# cv2.drawContours(frame, cnt, -1, color, -1)
			# cv2.drawContours(frame, cnt, -1, color, 12)
			cv2.addWeighted(frame, alpha, imgRgb, 1 - alpha, 0, imgRgb)
			#Draw and print the probability
			cv2.putText(imgRgb ,str(round(probabilitOfTomato[0],2)),(maxLoc[0],maxLoc[1]+5), 1, 1,(0,255,255),1,cv2.LINE_AA)
			# cv2.putText(imgRgb, "#{}".format(loopindex), (int(x) - 10, int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
			# cv2.circle(saliencyImage, maxLoc, 25, (0, 0, 0), -1)
			cv2.circle(imgRgb, maxLoc, 1, (255, 255, 0),-1)

			cv2.imshow("input",  imgRgb)
			cv2.imshow("saliencyImage",  saliencyImage)
			return  maxVal/255, saliencyImage, [x,y,r]
		else:
			print ('Object left: ', len(listOfObjects))
			print('All object found')
			return 0, saliencyImage, [0,0,0]

	else:
		print ('Object left: ', len(listOfObjects))
		print('All object found')
		return 0, saliencyImage, [0,0,0]


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
				cv2.circle(imgRgb, (x,y), 1, (255, 255, 0),-1)

				# cover the detected region in salinecy matplotlib
				cv2.drawContours(saliencyImage, cnt, -1, 0, -1)
				cv2.drawContours(saliencyImage, cnt, -1, 0, 25)
				cv2.imshow("input",  imgRgb)
				cv2.imshow("saliencyImage",  saliencyImage)
				return  maxVal/255, saliencyImage, [x,y,r]

			else:
				print('All object found')
				return 0, saliencyImage, [0,0,0]
		else:
			print ('Non-tomato !!')
			non_tomato_number +=1
			cv2.drawContours(saliencyImage, cnt, -1, 0, -1)
			cv2.drawContours(saliencyImage, cnt, -1, 0, 25)
			return  maxVal/255, saliencyImage, [0,0,0]
	else:
		print('unknow target')
		cv2.circle(saliencyImage, maxLoc, 10, 0 ,-1)
		return  maxVal/255, saliencyImage, [0,0,0]

def cropedRegionOfInterest(imageFullSize, targetPose, ratio_width, ratio_hieght, targetListInOneScene):
	cropedImage = cropTarget(imageFullSize,targetPose[0],targetPose[1],30, ratio_width, ratio_hieght, extraRaduis =0)
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

def rotateImage(image, angle):
	rows = image.shape[0]
	cols = image.shape[1]

	M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
	dst = cv2.warpAffine(image,M,(cols,rows))
	return dst

def main(imageName):
	# Load from file
	with open(filenameModel, 'rb') as file:
	    trainedModel = pickle.load(file)
	print (trainedModel)
	# read
	imageDir = imageName
	imageFullSize = cv2.imread(imageDir)
	h,w,c = imageFullSize.shape
	w_new = 640
	h_new = 420
	# print(h,w)
	ratio_width = w/w_new
	ratio_hieght = h/h_new
	if imageFullSize is None:
		print
		("No image load!!")
		exit()
	# imageFullSize = rotateImage(imageFullSize, 45)
	img = cv2.resize(imageFullSize,(640,420))
	temp_image = img.copy()
	# for i in range(3):
	# 	img = cv2.pyrDown(img)
	# initialize
	imgsize = img.shape
	img_width  = imgsize[1]
	img_height = imgsize[0]
	sm = pySaliencyMap.pySaliencyMap(img_width, img_height)
	# findCircule(img)
	# computation
	if FindTime:
		start = time.time()
	saliency_map = sm.SMGetSM(img)
	corrected_image = cv2.convertScaleAbs(saliency_map*255)
	# entropyCompute(corrected_image)
	corrected_image = cv2.GaussianBlur(corrected_image, (GaussianBlurRadius, GaussianBlurRadius), 2)
	cv2.rectangle(corrected_image, (0,0), (corrected_image.shape[1],corrected_image.shape[0]), (0,0,0), 5)
	SaliencyMap = corrected_image.copy()
	# thresh = cv2.threshold(corrected_image, 200, 255, cv2.THRESH_BINARY )[1]
	# cv2.imshow('Threshold', thresh)
	# cv2.waitKey(0)
	if FindTime == False:
		end = time.time()
		print ("Time to compute Saliency map: ", end - start)

	# if FindTime:
	# 	start = time.time()

	# listOfObjects = waterShedSaliencyMap(corrected_image, img, showImage =True)
	#
	# if FindTime == False:
	# 	end = time.time()
	# 	print ("Time to waterShed: ", end - start)
	probability = 1.0
	minimumPropapility = 0.8
	checkProbability = True
	targetListInOneScene = []
	index = 0
	savedImageIndex = 0
	# frame = np.zeros(corrected_image.shape, corrected_image.dtype)
	if FindTime:
		start = time.time()
	while(probability > minimumPropapility):
		# probability, corrected_image, targetPose = trackingSaliency(img, corrected_image, listOfObjects,  minimumPropapility,model= trainedModel,  loopindex= index)
		probability, corrected_image, targetPose = trackingSaliencyRegionBase(img, corrected_image, minimumPropapility,model= trainedModel,  loopindex= index)
		# print ('probability= ', probability, minimumPropapility)
		index += 1
		if targetPose != None:
			cropedRegionOfInterest(imageFullSize, targetPose, ratio_width, ratio_hieght, targetListInOneScene)
		if checkProbability:
			minimumPropapility =abs(probability - 0.3)
			checkProbability = False
		if LoopThroughAllImages:
			ikey = cv2.waitKey(1)
		else:
			ikey = cv2.waitKey(0)
		if ikey == ord('q'):
			exit()
		if ikey == ord('s'):
			ccc = cv2.convertScaleAbs(saliency_map*255)
			cv2.imwrite('imageOutPut.png', img)
			cv2.imwrite('imageOutPutSaliency.png', ccc)
	if FindTime:
		end = time.time()
		# print ("Loop through all points: ", end - start)
	print ('Total targets: ',len(targetListInOneScene))
	print('Non-tomato = ', non_tomato_number)
	if LoopThroughAllImages:
		splitName,_ = imageName.split('.')
		newImageName = 'output/'+ splitName + '_Total_Targets__'+  str(len(targetListInOneScene)) + '.png'
		saliencyFileName = 'Saliency/' + imageName
		_,imageNumber = imageName.split('/')
		newImageSizeSave = 'newImage/' + imageNumber
		print (newImageName)
		print (saliencyFileName)
		cv2.imwrite(newImageSizeSave, temp_image)
		cv2.imwrite(newImageName, img)
		cv2.imwrite(saliencyFileName, SaliencyMap)
	print ("Find all tomatos with propability bigger than % ",minimumPropapility*100 )
	ikey = cv2.waitKey(1)
	if LoopThroughAllImages == False:
		cv2.destroyAllWindows()

import os
if __name__ == '__main__':
	if LoopThroughAllImages:
		directory = 'flea3/'
		for filename in os.listdir(directory):
		    if filename.endswith(".png"):
		        imageName = os.path.join(directory, filename)
		        print (imageName)
		        main(imageName)
		    else:
		        continue
	else:
		main(imageName)
