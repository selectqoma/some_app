#!/usr/bin/env python3

import numpy as np
import cv2
import time
import pytest
import sys

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


@pytest.fixture
def node():
    rospy.init_node('rtsp_test', anonymous=True)


@pytest.fixture
def rtsp_tester():

    class RTSPTester:

        def __init__(self):
            self.img = None
            self.cv_bridge = CvBridge()

            # create publisher
            self.pub = rospy.Publisher('/test_image', Image, queue_size=1)

            # create subscriber
            self.sub = rospy.Subscriber('/test_image_from_rtsp', Image,
                                   self.callback)

            self.test_img = np.random.randint(
                0, 255, size=(640, 480, 3), dtype=np.uint8)

            # convert test image to ros message
            self.test_msg = self.cv_bridge.cv2_to_imgmsg(self.test_img)
            self.test_msg.header.stamp = rospy.Time.now()
            self.test_msg.encoding = "bgr8"


        def callback(self, msg):
            self.img = self.cv_bridge.imgmsg_to_cv2(msg)


        def wait_for_image(self, timeout):
            timeout_t = time.time() + timeout
            while not rospy.is_shutdown() and not self.received_image and \
                    time.time() < timeout_t:
                time.sleep(0.1)


        @property
        def received_image(self):
            if type(self.img) is np.ndarray:
                return True
            else:
                return False


        @property
        def same_image_shapes(self):
            return self.img.shape == self.test_img.shape


        def reset(self):
            self.img = None
            self.test_img = None
            self.test_msg = None


    return RTSPTester()


def test_transmission_and_receival(node, rtsp_tester):
    # delay 1 second to allow for nodes to initialize
    rospy.sleep(rospy.Duration(1.0))

    # publish test img msg
    rtsp_tester.pub.publish(rtsp_tester.test_msg)

    # wait for receival
    rtsp_tester.wait_for_image(20.0)

    assert rtsp_tester.received_image
    assert rtsp_tester.same_image_shapes
