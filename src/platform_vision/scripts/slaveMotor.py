#!/usr/bin/env python

import rospy
import cv2
import argparse
from platform_vision.SlaveCameraController import SlaveCameraController

ap = argparse.ArgumentParser(description='argument to control the slave controller!!')
ap.add_argument('-s', '--scale',type=int, default=0, help='This use to control the size of the image process')
ap.add_argument('-a', '--algorithm', default= 'PNCC', help='Chosse the algorithm to use PNCC, kaze, FLANN, Brute ')

args=vars(ap.parse_args())

ScaleDown = args['scale']
algorithm = args['algorithm']
def main(scale):
    rospy.init_node('SlaveMotorController', anonymous = True)
    slaveController = SlaveCameraController(activeTilitController=True, algorithmToUse = algorithm, scaleDown=ScaleDown)
    try:
        slaveController.trackObject()
        # rospy.spin()

    except KeyboardInterrupt:
        print "Shutting FastMatchingPyramid node down"
    cv2.destroyAllWindows()
    slaveController.moveToZero()

if __name__ == '__main__':
    scale = args['scale']
    print(scale)
    main(scale)
