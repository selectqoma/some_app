#!/usr/bin/env python3

import rospy
from smelter_osi_pi.osi_pi_proxy import OSIPIProxy


if __name__ == '__main__':
    rospy.init_node('osi_pi_proxy_node')
    OSIPIProxy()
    rospy.spin()
