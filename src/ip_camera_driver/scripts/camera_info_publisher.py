#!/usr/bin/env python2
''' alla '''

import os
import rospy
from sensor_msgs.msg import CameraInfo
from camera_calibration_parsers import readCalibration
import rospkg


class CameraInfoPublisher(object):
    ''' Class that reads a camera info file and publishes ros messages '''

    def __init__(self):
        rate = rospy.get_param('~rate')
        camera_name = rospy.get_param('~camera_name')
        self.frame_id = rospy.get_param('~frame_id')
        self.cam_info = self.read_camera_info(camera_name)
        self.cam_info_pub = rospy.Publisher('~camera_info', CameraInfo,
                                            queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.callback)

    def callback(self, _):
        ''' Timer callback that publishes the camera info message '''
        self.cam_info.header.stamp = rospy.Time.now()
        self.cam_info.header.seq += 1
        self.cam_info_pub.publish(self.cam_info)

    def read_camera_info(self, camera_name):
        ''' Creates and returns a camera info message from a yaml file

        Keyword Arguments:
        camera_name -- prefix of camera info file
        '''
        cam_info_path = os.path.join(
            rospkg.RosPack().get_path('ip_camera_driver'),
            'calibrations', camera_name+'_intrinsic_calibration.yaml')
        camera_info = None
        ret = readCalibration(cam_info_path)
        rospy.logerr(ret)
        camera_info = ret[1]
        try:
            camera_info.header.frame_id = self.frame_id
            rospy.loginfo(
                'Loaded camera calibration file for camera {} from path'
                ' {}'.format(camera_name, cam_info_path))
        except TypeError as err:
            rospy.logwarn('Failed to find a camera calibration file for '
                          'camera {} in path {} with error {}'.format(
                              camera_name, cam_info_path, err))
        return camera_info


if __name__ == '__main__':
    rospy.init_node('camera_info_publisher_node')
    CameraInfoPublisher()
    rospy.spin()
