# import the necessary packages
from skimage.feature import peak_local_max
from skimage.morphology import watershed, disk
from scipy import ndimage
import numpy as np
import cv2

from helpFunctions import *

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


def waterShedSaliencyMap(img,rgbImg,minRadius = 5,maxRadius = 80, showImage = False):
	colorImage = rgbImg.copy()
	thresh = img.copy()
	mask = np.zeros((img.shape[1], img.shape[0]))
	listOfObjects = []
	# noise removal
	(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(img)
	ratioThresholds = maxVal - maxVal * 0.5
	# thresh[thresh <  ratioThresholds ] = 0
	# thresh[thresh >  ratioThresholds ] = 255

	thresh = cv2.threshold(thresh, ratioThresholds, maxVal, cv2.THRESH_BINARY)[1]

	# kernel = np.ones((2,2),np.uint8)
	# thresh = cv2.erode(thresh,kernel,iterations=2)
	# Finding sure foreground area
	# dist_transform = cv2.distanceTransform(thresh,cv2.DIST_L2,5)
	# _, thresh = cv2.threshold(dist_transform,0.3*dist_transform.max(),255,0)
	cv2.imshow("Thresh___", thresh)

	# compute the exact Euclidean distance from every binary
	# pixel to the nearest zero pixel, then find peaks in this
	# distance map
	D = ndimage.distance_transform_edt(thresh, sampling=[3,3])
	localMax = peak_local_max(D, indices=False, min_distance=minRadius, labels=thresh)
	# cv2.imshow("DDD", D)
	# perform a connected component analysis on the local peaks,
	# using 8-connectivity, then appy the Watershed algorithm
	markers = ndimage.label(localMax, structure=np.ones((3, 3)))[0]
	labels = watershed(-D, markers, mask=thresh)
	# print("[INFO] {} unique segments found".format(len(np.unique(labels)) - 1))

	# loop over the unique labels returned by the Watershed
	# algorithm
	for label in np.unique(labels):
		# if the label is zero, we are examining the 'background'
		# so simply ignore it
		if label == 0:
			continue

		# otherwise, allocate memory for the label region and draw
		# it on the mask
		mask = np.zeros(img.shape, dtype="uint8")
		mask[labels == label] = 255
		# mask = cv2.erode(mask,kernel,iterations=2)
		# detect contours in the mask and grab the largest one
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		# print(len(cnts))
		# cv2.drawContours(mask, cnts, -1, (255,255,255))


		# print( cv2.contourArea(cnt) )
		# cv2.waitKey(0)
		c = max(cnts, key=cv2.contourArea)
		cnt = cnts[0]
		area = cv2.contourArea(c)
		# print(area)
		if area < 4673.0 and area > 60:
			# cv2.imshow("mask___", mask)
			# cv2.waitKey(0)
			# draw a circle enclosing the object
			((x, y), r) = cv2.minEnclosingCircle(c)
			# print ("#{}".format(label), r)
			if len(c) >= 5 and (r <= maxRadius and r >= minRadius):
				# cv2.putText(rgbImg, "#{}".format(label), (int(x) - 10, int(y)),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
				ellipse = cv2.fitEllipse(c)
				(x,y),(MA,ma),angle = cv2.fitEllipse(c)
				listOfObjects.append([int(np.sqrt(x*x+y*y)),ellipse, [x,y] ,r, cnts])
				# listOfObjectsPose.append([x,y])
				if showImage:
					color = np.random.randint(0,255,(3)).tolist()
					cv2.ellipse(mask, ellipse, 0, -1,cv2.LINE_AA)
					# cv2.circle(colorImage, (int(x), int(y)), int(r), color, -1)
					# cv2.drawContours(colorImage, c, -1, color, 5)
	# show the output image
	if showImage:
		# cv2.imshow("Thresh", img)
		cv2.imshow("mask", mask)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
	# print (len(listOfObjectsPose))
	# print (len(listOfObjects))
	return listOfObjects #, listOfObjectsPose
