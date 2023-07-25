#!/usr/bin/env python3

import rospy
from camera_patrol_controller.camera_patrol_controller import CameraPatrolController

if __name__ == '__main__':
    rospy.init_node('camera_patrol_controller_node')
    controller = CameraPatrolController()
    rospy.spin()
