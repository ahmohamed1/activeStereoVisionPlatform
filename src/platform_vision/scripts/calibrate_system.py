#!/usr/bin/env	python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np 


class Calibration:
	def	__init__(self,hoz_cor, vir_cor, board_num):

		self.hoz_cor = hoz_cor
		self.vir_cor = vir_cor
		self.board_num = board_num
		self.board_size = (self.hoz_cor, self.vir_cor)
		self.img_left_sata = False
		self.img_right_sata = False

		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.left_image_sub = rospy.Subscriber('stereo/left/image_raw',Image, self.left_img_callback)
		self.right_image_sub = rospy.Subscriber('stereo/right/image_raw',Image, self.right_img_callback)
		self.left_img = 0
		self.right_img = 0

		# Definet the Variables used in collecting the points of the calibration 
		self.left_img_point = []
		self.right_img_point = []

		# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
		self.object_point = np.zeros((self.hoz_cor* self.vir_cor,3), np.float32)
		self.object_point[:,:2] = np.mgrid[0:self.hoz_cor,0:self.vir_cor].T.reshape(-1,2)



	def	left_img_callback(self, msg):
		self.left_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		self.img_left_sata = True

	def	right_img_callback(self, msg):
		self.right_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		self.img_right_sata = True

	def show_image(self):
		while(True):
			#self.left_img = cv2.resize(self.left_img,(800,600))
			#self.right_img = cv2.resize(self.right_img,(800,600))
			#cv2.namedWindow("left_img",cv2.WINDOW_OPENGL)
			#cv2.namedWindow("right_img",cv2.WINDOW_OPENGL)
			cv2.imshow("left_img", self.left_img)
			cv2.imshow("right_img", self.right_img)
			ikey = cv2.waitKey(3)
			if ikey == ord('q'):
				exit()

	def find_corner(self):

		sucsses = 0
		while(sucsses < self.board_num):
			# Check if the image is not empty
			if self.img_right_sata == True & self.img_left_sata == True:
				# Convert image to gray 
				#left_img = cv2.resize(self.left_img,(800,600))
				#right_img = cv2.resize(self.right_img,(800,600))

				left_gray_img = cv2.cvtColor(self.left_img,cv2.COLOR_BGR2GRAY)
				right_gray_img = cv2.cvtColor(self.right_img,cv2.COLOR_BGR2GRAY)

				# Find corner 
				lret, left_corner = cv2.findChessboardCorners(left_gray_img, self.board_size,None)
				rret, right_corner = cv2.findChessboardCorners(right_gray_img, self.board_size,None)

				# If corners found in both image do a subPixel process
				if lret == True & rret == True:
					# termination criteria
					criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
					left_corner = cv2.cornerSubPix(left_gray_img, left_corner, (11, 11), (-1, -1), criteria)
					right_corner = cv2.cornerSubPix(right_gray_img, right_corner, (11, 11), (-1, -1), criteria)

				# Draw corner on image and dispaly
				left_img = cv2.drawChessboardCorners(left_img, self.board_size, left_corner, lret)
				right_img = cv2.drawChessboardCorners(right_img, self.board_size, right_corner, rret)
				cv2.imshow("left_img", left_img)
				cv2.imshow("right_img", right_img)
				ikey = cv2.waitKey(3)
				if ikey == ord('q'):
					exit()

				# Check if the corner found save them in ther store
				if lret == True & rret == True:
					self.left_img_point.append(left_corner)
					self.right_img_point.append(right_corner)
					sucsses +=1

		cv2.destroyAllWindows()
				
	def calibrate_system(self):
		cameraMatrix1 =None
		cameraMatrix2 = None
		distCoeffs1 = None
		distCoeffs2 = None
		R =None
		T = None
		E = None
		F = None
		stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
		stereocalib_flags = cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_SAME_FOCAL_LENGTH | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
		
		retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objectPoints=self.object_point , imagePoints1=self.left_img_point, 
																					imagePoints2=self.right_img_point, imageSize=(800,600), cameraMatrix1=cameraMatrix1,
																					distCoeffs1=distCoeffs1, cameraMatrix2=cameraMatrix2, distCoeffs2=distCoeffs2,
																					criteria=stereocalib_criteria, flags=stereocalib_flags)

		#(objectPoints, imagePoints1, imagePoints2, imageSize[, cameraMatrix1[, distCoeffs1[, cameraMatrix2[, distCoeffs2[, R[, T[, E[, F[, criteria[, flags]]]]]]]]]]) 

		rectify_scale = 0 # 0=full crop, 1=no crop
		R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (640, 480),R, T, rectify_scale)
				
		while(True):
			# Check if the image is not empty
			if self.img_right_sata == True & self.img_left_sata == True:
				left_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (640, 480), cv2.CV_16SC2)
				right_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (640, 480), cv2.CV_16SC2)

				left_img_remap = cv2.remap(pair.left_img, left_maps[0], left_maps[1], cv2.INTER_LANCZOS4)
				right_img_remap = cv2.remap(pair.right_img, right_maps[0], right_maps[1], cv2.INTER_LANCZOS4)

				stereo = cv2.createStereoBM(numDisparities=16, blockSize=9)
				disparity = stereo.compute(left_img_remap,right_img_remap)

				cv2.imshow("left_img", left_img_remap)
				cv2.imshow("right_img", right_img_remap)
				cv2.imshow("Disp", disparity)
				ikey = cv2.waitKey(3)
				if ikey == ord('q'):
					exit()



rospy.init_node('follower')
calibration = Calibration(7,5,15)
#calibration.show_image()
calibration.find_corner()
#calibration.calibrate_system()
rospy.spin()
	
