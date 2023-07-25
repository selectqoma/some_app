#!/usr/bin/env python3

import rospy
from cascade_state_monitoring.pot_filling_state_monitoring import PotFillingStateMonitor

if __name__ == '__main__':
    rospy.init_node('pot_filling_state_monitoring_node')
    rospy.loginfo('pot_filling_state_monitoring_node initialized')
    monitor = PotFillingStateMonitor()
    rospy.spin()
