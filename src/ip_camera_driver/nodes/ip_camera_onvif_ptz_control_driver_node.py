#!/usr/bin/env python3

import cv2
import rospy
from ip_camera_driver.onvif_ptz_control_driver import ONVIFPTZControlDriver

if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('ip_camera_ptz_control_driver_node.py')

    # Create a camera driver object
    driver = ONVIFPTZControlDriver()

    # Spin the node
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
