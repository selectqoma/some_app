#!/usr/bin/env python3

import cv2
import rospy
from ip_camera_driver.cv_camera_driver import CVCameraDriver

if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('ip_camera_driver_node', disable_signals=True)

    # Create a camera driver object
    driver = CVCameraDriver()

    # Spin the node
    try:
      rospy.spin()
    except KeyboardInterrupt:
      rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
