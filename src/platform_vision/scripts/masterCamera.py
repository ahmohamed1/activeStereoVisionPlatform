#!/usr/bin/env python

import rospy
import sys

import platform_vision.MasterCameraController as MasterCameraController
import argparse
ap = argparse.ArgumentParser(description='argument to control the slave controller!!')
ap.add_argument('-a', '--algorithm', default='aruco', help='Select which algorithm you want to use there are: aruco, PNCC, colorWithPNCC, saliency and color')
ap.add_argument('-m', '--suspendMotor',type=int, default=1 , help='This opption is to suspend the Motors(0 stop, 1 move)')
ap.add_argument('-s', '--ScaleDown',type=int, default=0 , help='This use to scal down the image size (0: False, 1: True)')
args=vars(ap.parse_args())


def main(argument='aruco', suspendMotor= True, scaleDown=0):
    print(suspendMotor)
    rospy.init_node('MotorController', anonymous = True)
    motorController = MasterCameraController.MasterCameraController(argument, suspendMotor= suspendMotor, saveData=False, ScaleDown=scaleDown)
    try:
        motorController.trackObject()

    except KeyboardInterrupt:
        print "Shutting ArucoFinder node down"
    motorController.moveToZero()

if __name__ == '__main__':
    argument = args['algorithm']
    suspendMotor = args['suspendMotor']
    scaleDown = args['ScaleDown']
    # print ('_____', (suspendMotor))
    main(argument, suspendMotor, scaleDown)
