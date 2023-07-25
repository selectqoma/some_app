import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
sys.dont_write_bytecode = True

class CvBridgeConverter:

    def __init__(self):
        self.cv_bridge = CvBridge()

    def img_msg_to_cv2(self, img_msg):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(img_msg)
            return img
        except CvBridgeError as e:
            rospy.logwarn('Failed to convert ros message to cv image with exception {}'.format(e))
            return None

    def cv2_to_img_msg(self, img):
        try:
            #  img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="passthrough")
            if len(img.shape) == 3:
                img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
            else:
                img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="mono8")
            return img_msg
        except CvBridgeError as e:
            rospy.logwarn('Failed to convert cv image to ROS message with exception {}'.format(e))
            return None
