
import cv2
import matplotlib.pyplot as plt
import saliency_map.pySaliencyMap as pySaliencyMap
from saliency_map.helpFunctions import *
from saliency_map.regionBasedSegment import *
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

import time

# construct the argument parse and parse the arguments
import argparse
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--radius", type = int, default=11,
	help = "radius of Gaussian blur; must be odd")
ap.add_argument("-i","--image", default='../data/73_Image.png',
    help= "image required to be process, Include the type of the image( e.g img.png, img.jpg)")

ap.add_argument("-l","--loop", type=int, default= 0,
    help= "image required to be process, Include the type of the image( e.g img.png, img.jpg)")
args = vars(ap.parse_args())

GaussianBlurRadius = args["radius"]
imageName = args['image']
LoopThroughAllImages = args['loop']
FindTime = True
saveTemplate = False
filenameModel = '/home/abdulla/dev/activeStereoVisionPlatform/src/saliency_map/config/linear_regressor.pkl' #'GaussianNaiveBayes.sav'
with open(filenameModel, 'rb') as file:
	trainedModel = pickle.load(file)
print (trainedModel)

def ComputeFOA(imageFullSize):
	ListOfTargetsInformation = []
	if imageFullSize is None:
		print('There is no image!!')
		exit()

	h,w,c = imageFullSize.shape
	w_new = 640
	h_new = 420
	# print(h,w)
	ratio_width = w/w_new
	ratio_hieght = h/h_new

	img = cv2.resize(imageFullSize,(640,420))
	imgsize = img.shape
	temp_image = img.copy()
	img_width  = imgsize[1]
	img_height = imgsize[0]
	sm = pySaliencyMap.pySaliencyMap(img_width, img_height)

	saliency_map = sm.SMGetSM(img)
	corrected_image = cv2.convertScaleAbs(saliency_map*255)

	corrected_image = cv2.GaussianBlur(corrected_image, (GaussianBlurRadius, GaussianBlurRadius), 2)
	cv2.rectangle(corrected_image, (0,0), (corrected_image.shape[1],corrected_image.shape[0]), (0,0,0), 5)

	probability = 1.0
	minimumPropapility = 0.8
	checkProbability = True
	targetListInOneScene = []
	index = 0
	savedImageIndex = 0

	while(probability > minimumPropapility):
		cropedImage = None
		# probability, corrected_image, targetPose = trackingSaliency(img, corrected_image, listOfObjects,  minimumPropapility,model= trainedModel,  loopindex= index)
		probability, corrected_image, targetPose, probabilitOfTomato = trackingSaliencyRegionBase(img, corrected_image, minimumPropapility,model= trainedModel,  loopindex= index)
		# print("targetPose ", targetPose)
		if targetPose != [0,0,0]:
			# cropedRegionOfInterest(imageFullSize, targetPose, ratio_width, ratio_hieght, targetListInOneScene, extraRaduis =10)
			cropedImage = cropedRegionOfInterest(temp_image, targetPose, 1, 1, targetListInOneScene, extraRaduis =10)
			if cropedImage != None:
				# cv2.imshow('cropedTarget', cropedImage)
				# cv2.waitKey(1)
				list_ = [index, probabilitOfTomato, targetPose[2], [targetPose[0], targetPose[1]], cropedImage]
				ListOfTargetsInformation.append(list_)
			index += 1
		# else:
		# 	print('This is not target')
		if checkProbability:
			minimumPropapility = abs(probability - 0.3)
			checkProbability = False
		cv2.imshow('FOA', img)
		ikey = cv2.waitKey(1)
		if ikey == ord('q'):
			exit()
		if ikey == ord('s'):
			ccc = cv2.convertScaleAbs(saliency_map*255)
			cv2.imwrite('../../data/imageOutPut.png', img)
			cv2.imwrite('../../data/imageOutPutSaliency.png', ccc)

		# add the information into a ListOfTargets
	print ('Total targets: ',len(targetListInOneScene))
	print ("Find all tomatos with propability bigger than % ",minimumPropapility*100 )
	ikey = cv2.waitKey(1)
	# cv2.destroyAllWindows()
	return ListOfTargetsInformation   #[['idea', '2D propability', '2D size', '2D pose' ]]
