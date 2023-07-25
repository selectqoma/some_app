#!/usr/bin/env python3

import numpy as np
import cv2
import time
import pytest
import sys

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


@pytest.fixture
def node():
    rospy.init_node('osi_pi_tags_test', anonymous=True)


@pytest.fixture
def tester():

    class OSIPITagsTest:

        def __init__(self):
            # create subscriber
            self.sub = rospy.Subscriber(
                '/diagnostics', DiagnosticArray, self.callback)
            self.diagnostics_msg = None


        def callback(self, diagnostics_msg):
            self.diagnostics_msg = diagnostics_msg

        def wait_for_diagnostics(self, timeout):
            ts = time.time()

            while not rospy.is_shutdown() and not self.diagnostics_received \
                    and time.time() - ts < timeout:
                time.sleep(1)

        @property
        def diagnostics_received(self):
            return self.diagnostics_msg is not None

        @property
        def all_tags_found(self):
            return self.diagnostics_msg.status[1].level == DiagnosticStatus.OK

    return OSIPITagsTest()


def test_successful_tags_reading(node, tester):
    tester.wait_for_diagnostics(timeout=3*60)

    assert tester.diagnostics_received

    status = tester.diagnostics_msg.status[0]

    assert status.name == 'slagsquare/osi_pi/pot_states/tag_web_ids'
    assert status.level == DiagnosticStatus.OK
