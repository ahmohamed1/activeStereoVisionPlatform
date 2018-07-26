#!/usr/bin/env python

import rospy
import numpy as np


# import local packags
from platform_vision.MasterCameraController import MasterCameraController
from platform_vision.SlaveCameraController import SlaveCameraController
# from saliency_map.SaliencyMap import pySaliencyMap
from platform_vision.helpFunctions import GetImageClass
from saliency_map.implementFOA import *


class Visual_Attention:

    def __init__(self):
        self.mainMap = [['idea', '2D propability', '2D size', '2D pose' , '3D propability', '3D pose']]
        self.ListOfTargets = []
        self.visitedTargets = []

        # Define the master camera image
        self.master_ImageCallBack = GetImageClass('right')

        self.visualAttention_mode = 'fix position' # 'explore'
        # define the saliency map
        # saliency_height = 640
        # saliency_width = 420
        # self.saliencyMap = pySaliencyMap(saliency_width, saliency_height)

        # Define the gaze controller for the master camera
        self.masterCameraController = MasterCameraController('PNCC', suspendMotor = True ,saveData=False, ScaleDown=True)
        self.slaveController = SlaveCameraController(activeTilitController=True, algorithmToUse ='PNCC', scaleDown=True)
        # Define the vergency controller for slave camera


    def action(self):
        while not rospy.is_shutdown():
            image = self.master_ImageCallBack.getImage()
            # Step 1: get the targets from the saliency
            if len(self.ListOfTargets) < 1:
                self.ListOfTargets = ComputeFOA(image)
            # Step 2: Gaze on the target with maximum probability2D
            # drop the target out and added to the visited target
            targetToTrack = self.ListOfTargets.pop()                #[['idea', '2D propability', '2D size', '2D pose' ]]
            self.masterCameraController.set_Template_Center(targetToTrack[3])
            self.masterCameraController.setTemplateSize(targetToTrack[2])
            self.masterCameraController.trackObject(visualAttention = True)
            # Step 3: wait the topic from vergency controller to store the pose of the targets
            slaveController.trackObject()
            # Step 4: Compute the disparity and process the point cloud measure the affordance of grasp

            # Setp 5: Update the MainMap with all data and publish the data



def main():
    visual_Attention = Visual_Attention()

    try:
        visual_Attention.action()

    except KeyboardInterrupt:
        print "Shutting visual_Attention node down"


if __name__ == '__main__':
    main()
