#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

update_pub = None

def update_callback(data):
    global update_pub
    update_pub.publish(Empty())

if __name__ == '__main__':
    rospy.init_node('update_monitor_node')
    update_pub = rospy.Publisher('/slagsquare/rgb/detected_pots/image_raw', Empty, queue_size=1)
    update_timer = rospy.Timer(rospy.Duration(2.0), update_callback)
    rospy.spin()
