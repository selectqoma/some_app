#!/usr/bin/env python3

import sys
import rospy
from ip_camera_driver.flir_cgi_driver_ros import FlirCGIDriverROS

if __name__ == '__main__':
    rospy.init_node('flir_cgi_driver_node', disable_signals=True)
    driver = FlirCGIDriverROS()
    rospy.spin()
