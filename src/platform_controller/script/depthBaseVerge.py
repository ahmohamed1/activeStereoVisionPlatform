#!/usr/bin/env python
import rospy
import vergincyDepthClass

def main():
    rospy.init_node('DrawTrackingSystem', anonymous = True)
    drawTrackingSystem = vergincyDepthClass.DrawTrackingSystem()
    try:
        drawTrackingSystem.updateDrawing()
        # rospy.spin()

    except KeyboardInterrupt:
        print "Shutting FastMatchingPyramid node down"

if __name__ == '__main__':
    main()
