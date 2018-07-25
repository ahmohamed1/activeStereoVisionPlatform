#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

import math
import graphics
import time

class DrawTrackingSystem:
    def __init__(self):

        self.baselineSubscriber = rospy.Subscriber("/baseline/position", Float64, self.baselineCallback)
        self.leftCameraAngle = rospy.Subscriber("/left/pan/angle", Float64, self.leftAngleCallback)
        self.rightCameraAngle = rospy.Subscriber("/right/pan/angle", Float64, self.rightAngleCallback)

        self.leftAngle = 19
        self.rightAngle = -20

        self.PI = 3.14159265359
        # Define the graphics system
        self.baseline = None
        self.leftLine = None
        self.rightLine = None
        self.txt = None
        self.win = graphics.GraphWin(width=500, height=500)
        self.win.setCoords(-250, -250, 250, 250)
        self.win.setBackground("yellow")
        self.targetX = 0
        self.targetY = 0
        self.actualBaseline = 50


    def __del__(self):
        self.win.close

    def baselineCallback(self, msg):
        self.actualBaseline = msg.data / 10

    def leftAngleCallback(self, msg):
        self.leftAngle = msg.data

    def rightAngleCallback(self, msg):
        self.rightAngle = msg.data


    def deg2rad(self,degree):
        return (degree * self.PI / 180);

    def calculateThePosition(self):
        B = self.actualBaseline;
        # Step 1- convert the angles to the coordinate
        AL = 90 - self.leftAngle
        AR = 90 + self.rightAngle

        # Step 2- calcualte the B angle and the length of r all values are in mm
        AO = 180 - (AL + AR)

        LS = B * math.sin(self.deg2rad(AL)) / math.sin(self.deg2rad(AO))
        RS = B * math.sin(self.deg2rad(AR)) / math.sin(self.deg2rad(AO))


        # Step 3 - calculate the targer position in x and z
        DR = RS * math.sin(self.deg2rad(AL))
        DL = LS * math.sin(self.deg2rad(AR))
        print  "depth R: " , DR,  " L: " , DL
        # Step 4 - calculate the short side therefore we can calculate the x position
        RSS = RS * math.cos(self.deg2rad(AL));
        LSS = LS * math.cos(self.deg2rad(AR));

        XposL = (-B/2) + LSS;
        XposR = (B/2) - RSS;

        # assinge the coordinate to the object position and publish the topic
        self.targetX = XposR
        self.targetY = DR

        print "X: ", XposL , " D: ", DR

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
        self.txt.draw(self.win)
        # time.sleep(0.09)

    def updateDrawing(self):
        rate = rospy.Rate(10) # 10hz
        # print "/////////////////////////////////////////////////////////////"
        while not rospy.is_shutdown():
            self.calculateThePosition()
            self.drawSystem()
            rate.sleep()


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
