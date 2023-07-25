#!/usr/bin/env python3

import rospy
from slagsquare_pot_instance_segmentation.pot_detector import PotDetector

if __name__ == '__main__':
    rospy.init_node('slagsquare_pot_detector_node')
    detector = PotDetector()
    rospy.spin()
