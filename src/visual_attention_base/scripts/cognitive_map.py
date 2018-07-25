#!/usr/bin/env python

import rospy
import numpy as np
import SaliencyMap


class Visual_Attention:

    def __init__(self):
        self.mainMap = [['idea', '2D propability', '2D size', '3D propability', '2D pose', '3D pose']]
        self.ListOfTargets = []
        self.visitedTargets = []


        self.visualAttention_mode = 'fix position' # 'explore'
        # define the saliency map
        self.saliency_image_size = [saliency_height, saliency_width]
        self.saliencyMap = SaliencyMap.SaliencyMap()

        # Define the gaze controller for the master camera


        # Define the vergency controller for slave camera


    def action(self):
        pass
        # Step 1: get the targets from the saliency

        # Step 2: Gaze on the target with maximum probability2D

        # drop the target out and added to the visited target

        # Step 3: wait the topic from vergency controller to store the pose of the targets

        # Step 4: Compute the disparity and process the point cloud measure the affordance of grasp

        # Setp 5: Update the MainMap with all data and publish the data

        
