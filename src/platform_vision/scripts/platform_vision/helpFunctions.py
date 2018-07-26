from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2


class GetImageClass:
    def __init__(self,topicName):
        self.topicName = '/stereo/' + topicName + '/image_color'
        self.image = None
        self.image_sub = rospy.Subscriber(self.topicName, Image, self.image_callback, queue_size=1) #/stereo/right/image_color
        self.bridge = CvBridge()

    def convertROSToCV(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            if self.ScaleDown:
                return cv2.resize(cv_image, (self.imageSize[0], self.imageSize[1]))
            else:
                return cv_image

        except CvBridgeError, e:
            print e

    def image_callback(self, data):
        self.image = self.convertROSToCV(data)

    def getImage(self):
        return self.image
