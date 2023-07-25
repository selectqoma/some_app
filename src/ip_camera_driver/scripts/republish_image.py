#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from threading import Lock

class ImageRepublisher:

    def __init__(self):
        rate = rospy.get_param("~rate", 10)
        self.cv_bridge = CvBridge()
        self.img = None
        self.img_lock = Lock()
        self.sub = rospy.Subscriber("~image", Image, self.sub_callback, queue_size=10)
        self.pub = rospy.Publisher("~republished_image", Image, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.pub_callback)

    def sub_callback(self, img_msg):
        with self.img_lock:
            self.img = img_msg

    def pub_callback(self, event):
        with self.img_lock:
            if self.img:
                self.pub.publish(self.img)

if __name__ == "__main__":
    rospy.init_node('republish_image_node')
    ir = ImageRepublisher()
    rospy.spin()
