#!/usr/bin/env python3

import cv2
import os
import rospkg
import rospy
import sys
import unittest
import numpy as np

from cascade_pot_detection.birdview_pot_detection import BirdviewPotDetector
from cascade_pot_detection.utils.cv_bridge_conversions import CvBridgeConverter

class TestPotDetection(unittest.TestCase):

    def test_camera_info_loading(self):
        pass

    def test_image_processing(self):
        pass

    def test_cv2_to_msg_conversion(self):
        img_path = rospkg.RosPack().get_path('cascade_pot_detection')
        img = cv2.imread(os.path.join(img_path, 'media', 'cascade_birdview_sample.jpg'))
        img_converter = CvBridgeConverter()
        img_msg = img_converter.cv2_to_img_msg(img)
        self.assertEqual(img_msg.height, 480)
        self.assertEqual(img_msg.width, 640)
        self.assertEqual(img_msg.encoding, 'bgr8')

    def test_msg_to_cv2_conversion(self):
        img_path = rospkg.RosPack().get_path('cascade_pot_detection')
        img = cv2.imread(os.path.join(img_path, 'media', 'cascade_birdview_sample.jpg'))
        img_converter = CvBridgeConverter()
        img_msg = img_converter.cv2_to_img_msg(img)
        new_img = img_converter.img_msg_to_cv2(img_msg)
        self.assertEqual(new_img.shape[0], 480)
        self.assertEqual(new_img.shape[1], 640)
        self.assertEqual(new_img.shape[2], 3)
        self.assertEqual(new_img.dtype, np.uint8)


    def test_pot_detection(self):
        pass


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('cascade_pot_detection', 'test_pot_detection', TestPotDetection)
