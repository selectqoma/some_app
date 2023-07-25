#!/usr/bin/env python3

import rospy
from smelter_diagnostics.diagnostics_publisher import DiagnosticsPublisher


if __name__ == '__main__':
    rospy.init_node('smelter_diagnostics_publisher_node')
    DiagnosticsPublisher()
    rospy.spin()
