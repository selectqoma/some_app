#!/usr/bin/env python3

import rospy
from smelter_osi_pi.slagsquare_pot_states_osi_pi_publisher \
    import PotStatesOSIPIPublisher

if __name__ == '__main__':
    rospy.init_node('slagsquare_pot_state_osi_pi_publisher_node')
    PotStatesOSIPIPublisher()
    rospy.spin()
