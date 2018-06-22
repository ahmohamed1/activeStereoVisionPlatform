#!/usr/bin/env python

import numpy as np
import cv2


class BaseFeatureMatching:

    def __init__(self,debug = False):
        print ('Feature matching algorithm')
        self.debug = debug

        self.algorithmDictionary = {'kaze': self.kaze_match,
                                    'FLANN': self.FLANNBasedMatcher,
                                    'Brute': self.BruteForceMatchingwithSIFTDescriptorsandRatioTest}


    def findCenterOfTarget(self, dst):
        a =  np.mean(dst, axis=0)
        return (a[0][0], a[0][1])


    def kaze_match(self, img1, img2):
        center = None
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        # initialize the AKAZE descriptor, then detect keypoints and extract
        # local invariant descriptors from the image
        detector = cv2.AKAZE_create()
        (kp1, descs1) = detector.detectAndCompute(gray1, None)
        (kp2, descs2) = detector.detectAndCompute(gray2, None)
        if self.debug:
            print("keypoints: {}, descriptors: {}".format(len(kp1), descs1.shape))
            print("keypoints: {}, descriptors: {}".format(len(kp2), descs2.shape))

        # Match the features
        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = bf.knnMatch(descs1,descs2, k=2)    # typo fixed

        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.9*n.distance:
                good.append(m)

        MIN_MATCH_COUNT = 5

        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = gray1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            # Find the center and draw the it
            center = self.findCenterOfTarget(dst)
            img2 = cv2.circle(img2,(center[0], center[1]),10,  (255,0,0), -1)
            img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

        else:
            print ("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
            matchesMask = None

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                           singlePointColor = None,
                           matchesMask = matchesMask, # draw only inliers
                           flags = 2)
        # cv2.drawMatchesKnn expects list of lists as matches.
        img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None)
        cv2.putText(img3,'AKAZE matching',(img3.shape[0]//2,50), 1, 5,(0,0,0),5,cv2.LINE_AA)
        return img3, center

    def FLANNBasedMatcher(self, img1,img2):
        center = None
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        # Initiate SIFT detector
        sift = cv2.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        MIN_MATCH_COUNT = 5
        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = gray1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            # Find the center and draw the it
            center = self.findCenterOfTarget(dst)
            img2 = cv2.circle(img2,(center[0], center[1]),10,  (255,0,0), -1)
            img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                           singlePointColor = None,
                           matchesMask = matchesMask, # draw only inliers
                           flags = 2)

        img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
        cv2.putText(img3,'FLANNBasedMatcher',(img3.shape[0]//2,50), 1, 5,(0,0,0),5,cv2.LINE_AA)

        # cv2.imshow("FLANNBasedMatcher", img3)
        # cv2.waitKey(10)
        return img3, center


    def BruteForceMatchingwithSIFTDescriptorsandRatioTest(self, img1,img2):
        center = None
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        # Initiate SIFT detector
        sift = cv2.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(gray1,None)
        kp2, des2 = sift.detectAndCompute(gray2,None)

        # BFMatcher with default params
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2, k=2)

        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append(m)

        MIN_MATCH_COUNT = 10
        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            # center = np.mean(dst_pts, axis=0)
            # print center[0]
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = gray1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            center = self.findCenterOfTarget(dst)
            # print (center)
            img2 = cv2.circle(img2,(center[0], center[1]),10,  (255,0,0), -1)
            img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                           singlePointColor = None,
                           matchesMask = matchesMask, # draw only inliers
                           flags = 2)

        # cv2.drawMatchesKnn expects list of lists as matches.
        # img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None)
        img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
        cv2.putText(img3,'BruteForceMatching',(img3.shape[0]//2,50), 1, 5,(0,0,0),5,cv2.LINE_AA)
        # cv2.imshow("Brute Force Matching", img3)
        # cv2.waitKey(3)
        return img3, center
