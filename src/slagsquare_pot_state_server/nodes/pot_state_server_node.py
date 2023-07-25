#!/usr/bin/env python3

import rospy
from slagsquare_pot_state_server.pot_state_server import PotStateServer

if __name__ == '__main__':
    rospy.init_node('slagsquare_pot_state_server_node')
    server = PotStateServer()
    rospy.spin()
