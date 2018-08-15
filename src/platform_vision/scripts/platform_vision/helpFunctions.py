from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2
import numpy as np

_magic = [0.299, 0.587, 0.114]
_zero = [0, 0, 0]
_ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

true_anaglyph = ([_magic, _zero, _zero], [_zero, _zero, _magic])
gray_anaglyph = ([_magic, _zero, _zero], [_zero, _magic, _magic])
color_anaglyph = ([_ident[0], _zero, _zero], [_zero, _ident[1], _ident[2]])
half_color_anaglyph = ([_magic, _zero, _zero], [_zero, _ident[1], _ident[2]])
optimized_anaglyph = ([[0, 0.7, 0.3], _zero, _zero], [_zero, _ident[1], _ident[2]])
methods = [true_anaglyph, gray_anaglyph, color_anaglyph, half_color_anaglyph, optimized_anaglyph]

class GetImageClass:
    def __init__(self,topicName):
        self.topicName = '/stereo/' + topicName + '/image_color'
        self.image = None
        self.image_sub = rospy.Subscriber(self.topicName, Image, self.image_callback, queue_size=1) #/stereo/right/image_color
        self.bridge = CvBridge()

    def convertROSToCV(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            return cv_image

        except CvBridgeError, e:
            print e

    def image_callback(self, data):
        self.image = self.convertROSToCV(data)

    def getImage(self):
        return self.image


class CreateAnaglyphImage:
    def __init__(self):
        self._magic = [0.299, 0.587, 0.114]
        self._zero = [0, 0, 0]
        self._ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        self.true_anaglyph = ([self._magic, self._zero, self._zero], [self._zero, self._zero, self._magic])
        self.gray_anaglyph = ([self._magic, self._zero, self._zero], [self._zero, self._magic, self._magic])
        self.color_anaglyph = ([self._ident[0], self._zero, self._zero], [self._zero, self._ident[1], self._ident[2]])
        self.half_color_anaglyph = ([self._magic, self._zero, self._zero], [self._zero, self._ident[1], self._ident[2]])
        self.optimized_anaglyph = ([[0, 0.7, 0.3], self._zero, self._zero], [self._zero, self._ident[1], self._ident[2]])
        self.methods = [self.true_anaglyph, self.gray_anaglyph, self.color_anaglyph, self.half_color_anaglyph, self.optimized_anaglyph]

    # http://bytes.com/topic/python/answers/486627-anaglyph-3d-stereo-imaging-pil-numpy
    def anaglyph(self, image1, image2, method=true_anaglyph):

        m1, m2 = [np.array(m).transpose() for m in method]
        image1 = np.dot(image1, m1) # float64
        image2 = np.dot(image2, m2) # int64
        composite = cv2.add(np.asarray(image1, dtype="uint8"), np.asarray(image2, dtype="uint8"))
        return composite
