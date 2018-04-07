#!/usr/bin/python
import roslib
roslib.load_manifest('stereo_anaglyph')
import rospy
import hrl_camera.ros_camera as rc
import cv2
from sensor_msgs.msg import Image


right_image = None
left_image = None

def convertROSToCV( data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        return cv_image, True
    except CvBridgeError, e:
        print e

def left_image_callback( data):
    left_image, left_image_state = convertROSToCV(data)

def right_image_callback(data):
    right_image, right_image_state = convertROSToCV(data)

left_image_sub = rospy.Subscriber('/stereo/left/image_color', Image,left_image_callback)
right_image_sub = rospy.Subscriber('/stereo/right/image_color', Image, right_image_callback)


def anaglyph(left_color, right_color):
    Bl, Gl, Rl = cv2.split(left_color)
    Br, Gr, Rr = cv2.split(right_color)
    result = cv2.merge((Bl*0.5,0,Rr*0.5))
    return result

def main:
    rospy.init_node('ArucoFinder', anonymous = True)
    while not rospy.is_shutdown():
        if left_image is not None and right_image is not None:
            red_blue = anaglyph(l, r)
            cv2.ShowImage('stereo-anaglyph', red_blue)
            cv2.WaitKey(10)


if __name__ == '__main__':
    main()
