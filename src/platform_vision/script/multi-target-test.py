#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int64

class MultiTargetSaver:

    def __init__(self):
        self.originIntialPosetion = None
        self.editedTargetsPosetion = None
        self.targetPosetion = np.array([[0, 0, 0]])
        self.targetPosetionStorage = np.zeros((16,3))
        self.targetPositionSub = rospy.Subscriber("targetPose", Vector3, self.targetPoseCallback,queue_size=1)
        self.ideaToTrackPub = rospy.Publisher('/ideaNumber',Int64, queue_size=1)
        self.ideaToTrack = 18
        self.originalPosition = np.array([[-15,15,12.7],
                             [-5,15,5],
                             [5,15,8.2],
                             [15,15,12.3],
                             [-15,5,6.2],
                             [-5,5,7.6],
                             [5,5,11.6],
                             [15,5,9.4],
                             [-15,-5,12.2],
                             [-5,-5,8],
                             [5,-5,5],
                             [15,-5,9.4],
                             [-15,-15,6],
                             [-5,-15,6.4],
                             [5,-15,8.1],
                             [15,-15,10.1]])

    def targetPoseCallback(self, data):
        self.targetPosetion = np.array([[data.x, data.y, data.z]])
        if self.ideaToTrack == 18:
            self.originIntialPosetion = self.targetPosetion
            self.editedTargetsPosetion = self.originIntialPosetion - self.originalPosition


    def publishIdeaToTrack(self,idea):
        targetIdea = Int64()
        targetIdea.data = idea
        self.ideaToTrackPub.publish(targetIdea)
        self.ideaToTrack 

    def storeData(self, pose, ide):
        self.targetPosetionStorage[ide - 1] = self.originIntialPosetion[ide - 1] - pose
        # increase the idea

    def mainLoop(self):
        rate = rospy.Rate(15) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            text = "Idea: " + str(self.ideaToTrack) + str(self.targetPosetion)
            #Create empty image
            image = np.zeros((512,512,3), np.uint8)
            font = cv2.FONT_HERSHEY_SIMPLEX
            image = cv2.putText(image,text,(10,512/2), font, 1,(255,255,255),2,cv2.LINE_AA)
            cv2.imshow("Posetion", image)
            ikey = cv2.waitKey(10)
            if ikey == ord('s'):
                # Store data in the array
                self.storeData(self.targetPosetion, self.ideaToTrack)
            elif ikey == ord('q'):
                exit()


def main():
    rospy.init_node('ArucoFinder', anonymous = True)
    multiTargetSaver = MultiTargetSaver()
    try:
        multiTargetSaver.mainLoop()

    except KeyboardInterrupt:
        print "Shutting ArucoFinder node down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
