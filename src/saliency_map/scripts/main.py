
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
filenameModel = '../config/linear_regressor.pkl' #'GaussianNaiveBayes.sav'


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
		probability, corrected_image, targetPose, probabilitOfTomato = trackingSaliencyRegionBase(img, corrected_image, minimumPropapility,model= trainedModel,  loopindex= index)
		# print ('probability= ', probability, minimumPropapility)
		index += 1
		if targetPose != None:
			# cropedRegionOfInterest(imageFullSize, targetPose, ratio_width, ratio_hieght, targetListInOneScene, extraRaduis =10)
			cropedRegionOfInterest(temp_image, targetPose, 1, 1, targetListInOneScene, extraRaduis =10)
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
