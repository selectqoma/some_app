#!/usr/bin/env python3

import rospy
from slagsquare_monitoring_fsm.slagsquare_monitoring_fsm import SlagsquareMonitoringFSM

def main():
    rospy.init_node('slagsquare_monitoring_fsm_node')
    fsm = SlagsquareMonitoringFSM()
    fsm.spawn_introspection_server()
    fsm.execute()
    rospy.spin()
    fsm.stop_introspection_server()

if __name__ == '__main__':
    main()
