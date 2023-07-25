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
    rospy.init_node('milestone_streams_test', anonymous=True)


@pytest.fixture
def tester():

    class MilestoneStreamsTester:

        def __init__(self):
            self.cascade_img = None
            self.slagsquare_img = None
            self.cv_bridge = CvBridge()

            # create publishers
            self.cascade_pub = rospy.Publisher(
                'cascade/streamer/image_raw', Image, queue_size=1)
            self.slagsquare_pub = rospy.Publisher(
                'slagsquare/streamer/image_raw', Image, queue_size=1)

            # create subscriber
            self.cascade_sub = rospy.Subscriber(
                'cascade/capture/image_raw', Image, self.cascade_callback)
            self.slagsquare_pub = rospy.Subscriber(
                'slagsquare/capture/image_raw', Image,
                self.slagsquare_callback)

            # create test image
            cols, rows = 960, 640
            self.test_img = np.zeros((rows, cols, 3), dtype=np.uint8)
            dx, dy = cv2.getTextSize(
                text='TEST', fontFace=cv2.FONT_HERSHEY_DUPLEX, thickness=1)
            pos = (cols // 2 - dx // 2, rows // 2 + dy // 2)
            cv2.putText(img, 'TEST', pos, cv2.FONT_HERSHEY_DUPLEX, 1,
                        (255,)*3, 1, lineType=cv2.LINE_AA)

            # convert test image to ros message
            self.test_msg = self.cv_bridge.cv2_to_imgmsg(self.test_img)
            self.test_msg.header.stamp = rospy.Time.now()
            self.test_msg.encoding = "bgr8"


        def cascade_callback(self, msg):
            self.cascade_img = self.cv_bridge.imgmsg_to_cv2(msg)


        def slagsquare_callback(self, msg):
            self.slagsquare_img = self.cv_bridge.imgmsg_to_cv2(msg)


        def wait_for_images(self, timeout):
            timeout_t = time.time() + timeout
            while not rospy.is_shutdown() and not self.received_images and \
                    time.time() < timeout_t:
                time.sleep(0.1)


        @property
        def received_images(self):
            if type(self.cascade_img) is np.ndarray and \
                    type(self.slagsquare_img) is np.ndarray:
                return True
            else:
                return False


        @property
        def same_image_shapes(self):
            return self.cascade_img.shape == self.slagsquare_img.shape == \
                self.test_img.shape

    return RTSPTester()


def test_milestone_streams(node, tester):
    # delay 1 second to allow for nodes to initialize
    rospy.sleep(rospy.Duration(1.0))

    # publish test img msg
    tester.cascade_pub.publish(tester.test_msg)
    tester.slagsquare_pub.publish(tester.test_msg)

    # wait for receival
    tester.wait_for_images(20.0)

    assert tester.received_images
    assert tester.same_image_shapes
