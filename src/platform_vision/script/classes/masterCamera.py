#!/usr/bin/env python

import rospy
import sys

import MasterCameraController

def main(argument='aruco', suspendMotor= True):
    rospy.init_node('MotorController', anonymous = True)
    motorController = MasterCameraController.MasterCameraController(argument, suspendMotor= suspendMotor, saveData=False)
    try:
        motorController.trackObject()

    except KeyboardInterrupt:
        print "Shutting ArucoFinder node down"
    motorController.moveToZero()

if __name__ == '__main__':
    argument = sys.argv[1]
    suspendMotor = sys.argv[2].lower() == 'true'
    # print ('_____', (suspendMotor))
    main(argument, suspendMotor)
