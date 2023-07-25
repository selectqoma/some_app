#!/usr/bin/env python3

import rospy
from slagsquare_groundplan_visualization.groundplan_visualization import GroundplanVisualizer

if __name__ == '__main__':
    rospy.init_node('slagsquare_groundplan_visualization_node')
    visualizer = GroundplanVisualizer()
    rospy.spin()
