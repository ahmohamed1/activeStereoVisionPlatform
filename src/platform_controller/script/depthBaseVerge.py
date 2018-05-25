#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

import numpy as np
import math
import graphics
import time
#Because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

class DrawTrackingSystem:
    def __init__(self):

        self.baselineSubscriber = rospy.Subscriber("/baseline/position", Float64, self.baselineCallback)
        self.leftCameraAngle = rospy.Subscriber("/left/pan/angle", Float64, self.leftAngleCallback)
        self.rightCameraAngle = rospy.Subscriber("/right/pan/angle", Float64, self.rightAngleCallback)
        self.tiltAnglgeSibscribtion = rospy.Subscriber("/right/tilt/angle", Float64, self.tiltAngleCallback)

        self.targetPositionPub = rospy.Publisher("targetPose", Vector3, queue_size=1)
        self.leftAngle = 19
        self.rightAngle = -20

        self.PI = 3.14159265359
        # Define the graphics system
        self.baseline = 0.0
        self.leftLine = 0.0
        self.rightLine = 0.0
        self.txt = 0.0
        self.win = graphics.GraphWin(width=500, height=500)
        self.win.setCoords(-250, -250, 250, 250)
        self.win.setBackground("yellow")
        self.targetX = 0
        self.targetY = 0
        self.actualBaseline = 50

        self.tiltingAngle = 0.0

        self.saveDataAble = False
        self.TrackingData = [['X','Y','Z']]

        # self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = "/camera_link"
        self.t.header.stamp = rospy.Time.now()
        self.t.child_frame_id = 'target_1'


    def dynamicTFBroadcaster(self, _X, _Y,_Z):
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = _X/100
        self.t.transform.translation.y = _Y/100
        self.t.transform.translation.z = _Z/100

        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0

        self.br.sendTransform(self.t)

    def publishPose(self, X, Y, Z):
        pose = Vector3()
        pose.x = X
        pose.y = Y
        pose.z = Z
        self.targetPositionPub.publish(pose)

    def __del__(self):
        self.win.close

    def baselineCallback(self, msg):
        self.actualBaseline = (msg.data / 10) + 0.4

    def leftAngleCallback(self, msg):
        self.leftAngle = round(msg.data,2) #- 0.5

    def rightAngleCallback(self, msg):
        self.rightAngle = round(msg.data,2) #+ 0.5


    def tiltAngleCallback(self, msg):
        self.tiltingAngle = self.deg2rad(msg.data)
        # print self.tiltingAngle

    def deg2rad(self,degree):
        return (degree * self.PI / 180);

    def calculateThePosition(self):
        B = self.actualBaseline;
        # Step 1- convert the angles to the coordinate
        AL = 90 - self.leftAngle
        AR = 90 + self.rightAngle

        # Step 2- calcualte the B angle and the length of r all values are in mm
        AO = 180 - (AL + AR)
        if math.sin(self.deg2rad(AO)) != 0:
            LS = B * math.sin(self.deg2rad(AL)) / math.sin(self.deg2rad(AO))
            RS = B * math.sin(self.deg2rad(AR)) / math.sin(self.deg2rad(AO))
        else:
            LS = B * math.sin(self.deg2rad(AL)) / 0.01
            RS = B * math.sin(self.deg2rad(AR)) / 0.01

        # Step 3 - calculate the targer position in x and z
        DR = RS * math.sin(self.deg2rad(AL))
        DL = LS * math.sin(self.deg2rad(AR))
        # print  "depth R: " , DR,  " L: " , DL
        # Step 4 - calculate the short side therefore we can calculate the x position
        RSS = RS * math.cos(self.deg2rad(AL));
        LSS = LS * math.cos(self.deg2rad(AR));

        XposL = (-B/2) + LSS;
        XposR = (B/2) - RSS;

        # assinge the coordinate to the object position and publish the topic
        if (AL > AR ):
            self.targetX = XposL
            self.targetY = DR
            # self.targetY = (0.9346*(math.pow(DL,1.0214)))
        else:
            self.targetX = XposR
            self.targetY = DR
            # self.targetY = (0.9346*(math.pow(DR,1.0214)))

        x ,y, z = self.transformCoordinate(self.targetY, self.targetX, 0.0, -self.tiltingAngle)

        # depthTan = B / (math.tan(self.deg2rad(AR)) + math.tan(self.deg2rad(AL)))
        # print "X:", round(x,2) , " Y:", round(y,2) , " Z:", round(z,2)# , " DR: ", round(DR,1), " DL: ", round(DL,1)#, "Depth Based Tan: ", depthTan
        print "XR:", round(XposR,2) , " YR:", round(DR,2), " XL:", round(XposL,2) , " YL:", round(DL,2)
        self.publishPose(x,y,z)
        self.dynamicTFBroadcaster(x, y, z)

        if self.saveDataAble == True:
            self.TrackingData.append([x,y,z])

    def transformCoordinate(self, x, y, z, tiltingAngle):
        Rotation_y = np.matrix([ [np.cos(tiltingAngle) , 0.0 , np.sin(tiltingAngle)],
                                 [0.0                  , 1.0 , 0.0                 ],
                                 [-np.sin(tiltingAngle), 0.0 , np.cos(tiltingAngle)]])
        currentPose = np.matrix([[x],[y],[z]])

        newPose = Rotation_y * currentPose

        return newPose[0], newPose[1], newPose[2]

    def drawSystem(self):
        leftMotorPoint = graphics.Point(-self.actualBaseline/2, 0)
        rightMotorPoint = graphics.Point(self.actualBaseline/2, 0)
        targetPoint = graphics.Point(self.targetX , self.targetY)

        if self.baseline: # Delecte the old line to draw the new line
            self.baseline.undraw()
            self.leftLine.undraw()
            self.rightLine.undraw()
            self.txt.undraw()

        self.baseline = graphics.Line(leftMotorPoint, rightMotorPoint)
        self.leftLine = graphics.Line(leftMotorPoint, targetPoint)
        self.rightLine = graphics.Line(rightMotorPoint, targetPoint)

        # set the color of the lines
        self.baseline.setOutline(graphics.color_rgb(0,0,255))
        self.rightLine.setOutline(graphics.color_rgb(0,0,255))
        self.leftLine.setOutline(graphics.color_rgb(0,0,255))

        # Set width
        self.baseline.setWidth(3)
        self.rightLine.setWidth(3)
        self.leftLine.setWidth(3)

        self.leftLine.draw(self.win)
        self.rightLine.draw(self.win)
        self.baseline.draw(self.win)

        # Put text in the win
        coord = 'X:' + str("%.2f" % self.targetX) + ' Y:' + str("%.2f" % self.targetY)
        self.txt = graphics.Text(graphics.Point(self.targetX, self.targetY + 20), coord)
        self.txt.setSize(25)
        self.txt.setStyle("bold")
        self.txt.draw(self.win)
        # time.sleep(0.09)

    def updateDrawing(self):
        rate = rospy.Rate(10) # 10hz
        # print "/////////////////////////////////////////////////////////////"
        while not rospy.is_shutdown():
            self.calculateThePosition()
            self.drawSystem()
            rate.sleep()

        self.saveDateAfterFinish()

    def saveDateAfterFinish(self):
        import csv
        import os
        name = '/home/abdulla/TrackingSpeed/depthData.csv'
        fileBool = os.path.isfile(name)    # False
        if fileBool == False:
            with open(name, "wb") as f:
                writer = csv.writer(f)
                writer.writerows(self.TrackingData)
                print('Data saved correctely')



def main():
    rospy.init_node('DrawTrackingSystem', anonymous = True)
    drawTrackingSystem = DrawTrackingSystem()
    try:
        drawTrackingSystem.updateDrawing()
        # rospy.spin()

    except KeyboardInterrupt:
        print "Shutting FastMatchingPyramid node down"

if __name__ == '__main__':
    main()
