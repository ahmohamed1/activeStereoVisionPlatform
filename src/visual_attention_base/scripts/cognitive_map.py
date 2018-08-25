#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3


# import local packags
from platform_vision.MasterCameraController import MasterCameraController
from platform_vision.SlaveCameraController import SlaveCameraController
# from saliency_map.SaliencyMap import pySaliencyMap
from platform_vision.helpFunctions import GetImageClass
from saliency_map.implementFOA import *

from visual_attention_base.markerClass import CreateVisualizationMarker

class Visual_Attention:

    def __init__(self):
        self.mainMap = [] #['idea', '2D propability', '2D size', '2D pose' , '3D propability', '3D pose']
        self.ListOfTargets = []
        self.visitedTargets = []

        # Define the master camera image
        self.master_ImageCallBack = GetImageClass('left')
        self.slave_ImageCallBack = GetImageClass('right')


        self.visualAttention_mode = 'fix position' # 'explore'
        # define the saliency map
        # saliency_height = 640
        # saliency_width = 420
        # self.saliencyMap = pySaliencyMap(saliency_width, saliency_height)

        # Define the gaze controller for the master camera
        self.masterCameraController = MasterCameraController('PNCC', suspendMotor = True ,saveData=False, ScaleDown=True, visualAttention=True)

        # Define the vergency controller for slave camera
        # self.slaveController = SlaveCameraController(activeTilitController=True, algorithmToUse ='PNCC', scaleDown=True)
        # Define the markerClass
        self.createVisualizationMarker = CreateVisualizationMarker('camera_link')
        self.OnTargetSubscriver = rospy.Subscriber('/onTarget', Bool, self.vergeCallBack)
        self.targetPositionSubscriber = rospy.Subscriber("targetPose", Vector3, self.targetPoseCallback)
        self.vergeStatus = False
        self.TargetPose = [0,0,0]
        # Reset the system to ZERO
        self.masterCameraController.moveToZero()
        # self.slaveController.moveToZero()

    def moveToZero(self):
        # Reset the system to ZERO
        self.masterCameraController.moveToZero()
        # self.slaveController.moveToZero()

    def vergeCallBack(self, data):
        self.vergeStatus = data.data

    def targetPoseCallback(self, pose):
        self.TargetPose[0] = pose.x
        self.TargetPose[1] = pose.y
        self.TargetPose[2] = pose.z

    def checkVerge(self):
        vergeStatus = False
        while (vergeStatus == False) and not rospy.is_shutdown():
            vergeStatus = self.vergeStatus

    def action(self):
        while not rospy.is_shutdown():
            image = self.master_ImageCallBack.getImage()
            # Step 1: get the targets from the saliency
            if len(self.ListOfTargets) == 0:
                print("Computing the saliency map ====>")
                # Reset the system to ZERO
                self.moveToZero()

                self.ListOfTargets = ComputeFOA(image)
                print("Tonatal Target found : ", len(self.ListOfTargets))

            if len(self.ListOfTargets) != 0:
                # Step 2: Gaze on the target with maximum probability2D
                # drop the target out and added to the visited target
                targetToTrack = self.ListOfTargets.pop()     #[['idea', '2D propability', '2D size', '2D pose', 'templateImage']]
                print("Gazing on target ====>" )
                # cv2.imshow("template", targetToTrack[4])
                # cv2.waitKey(0)
                # self.masterCameraController.set_Template_Center(targetToTrack[3][0],targetToTrack[3][1])
                # self.masterCameraController.setTemplateSize(targetToTrack[2])
                self.masterCameraController.setTemplateImage(targetToTrack[4])
                self.masterCameraController.trackObject(visualAttention = True)

                # Step 3: wait the topic from vergency controller to store the pose of the targets
                print("Verging on target  ====>" )
                # pose3D = self.slaveController.trackObject(True)
                self.checkVerge()
                pose3D = self.TargetPose
                print ("3D pose: ", pose3D)
                # Step 4: Compute the disparity and process the point cloud measure the affordance of grasp
                propability3D = 'pointCloud'

                # Setp 5: Update the MainMap with all data and publish the data
                print("Updating the Main Saliency Map ====>")
                mainMap_tmp = [targetToTrack[0], targetToTrack[2], targetToTrack[3], propability3D, pose3D]
                #['idea', '2D propability', '2D pose' , '3D propability', '3D pose']
                # mainMap_tmp = [2, 0.78, [350,350], '0.86', [1,.05,1.03]]
                self.mainMap.append(mainMap_tmp)
                self.createVisualizationMarker.publishMarkerArray(self.mainMap)



def main():
    rospy.init_node('Cognitive_map', anonymous = False)
    visual_Attention = Visual_Attention()
    try:
        visual_Attention.action()
        visual_Attention.moveToZero()
    except KeyboardInterrupt:
        print "Shutting visual_Attention node down"


if __name__ == '__main__':
    main()
