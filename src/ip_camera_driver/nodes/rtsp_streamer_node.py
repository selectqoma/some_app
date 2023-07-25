#!/usr/bin/env python3

from ip_camera_driver.rtsp_streamer import FrameStreamer
import rospy

def main():
    rospy.init_node('rtsp_streamer_node', anonymous=True)
    streamer = FrameStreamer()
    rospy.spin()

if __name__ == '__main__':
    main()
