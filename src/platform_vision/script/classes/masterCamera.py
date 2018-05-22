#!/usr/bin/env python

import rospy
import sys

import MasterCameraController
import argparse
ap = argparse.ArgumentParser(description='argument to control the slave controller!!')
ap.add_argument('-a', '--algorithm', default='aruco', help='Select which algorithm you want to use there are: aruco, PNCC, colorWithPNCC, saliency and color')
ap.add_argument('-m', '--suspendMotor',type=int, default=1 , help='This opption is to suspend the Motors(0 stop, 1 move)')
args=vars(ap.parse_args())


def main(argument='aruco', suspendMotor= True):
    print(suspendMotor)
    rospy.init_node('MotorController', anonymous = True)
    motorController = MasterCameraController.MasterCameraController(argument, suspendMotor= suspendMotor, saveData=False, ScaleDown=True)
    try:
        motorController.trackObject()

    except KeyboardInterrupt:
        print "Shutting ArucoFinder node down"
    motorController.moveToZero()

if __name__ == '__main__':
    argument = args['algorithm']
    suspendMotor = args['suspendMotor']
    # print ('_____', (suspendMotor))
    main(argument, suspendMotor)
