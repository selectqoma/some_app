import rospy
from sensor_msgs.msg import Image
from utils.cv_bridge_conversions import CvBridgeConverter
import sys
sys.dont_write_bytecode = True

class PotDetector():
    def __init__(self):
        self.img_sub = rospy.Subscriber('~image', Image, self.image_cb, queue_size=1)
        self.annot_img_pub = rospy.Publisher('~detected_pots/image', Image, queue_size=1)
        self.img_converter = CvBridgeConverter()

    def image_cb(self, data):
        # convert to cv2 using cv_bridge
        img = self.img_converter.img_msg_to_cv2(data)

        # detect pots
        rois, annot_img = self.detect_pots(img)

        # convert back to Image msg and publish annotated image
        annot_img_msg = self.img_converter.cv2_to_img_msg(annot_img)
        self.annot_img_pub.publish(annot_img_msg)

    def detect_pots(self, img):
        rospy.loginfo('Using default pot detection function that returns the same image and not rois')
        return None, img
