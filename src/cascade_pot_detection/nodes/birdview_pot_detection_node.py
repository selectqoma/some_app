#!/usr/bin/env python3

import rospy
from pot_detection.cascade_pot_detection.birdview_pot_detection import BirdviewPotDetector

if __name__ == '__main__':
    rospy.init_node('birdview_pot_detection_node')
    rospy.loginfo('birdview_pot_detector_node initialized')
    detector = BirdviewPotDetector()
    rospy.spin()
