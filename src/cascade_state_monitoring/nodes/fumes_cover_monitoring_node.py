#!/usr/bin/env python3

import os
import cv2
import sys
from argparse import ArgumentParser
import rospy
import numpy as np
from rospkg import RosPack
from cascade_state_monitoring.fumes_cover_monitor import FumesCoverMonitor

import pdb

if __name__ == '__main__':
    rospy.init_node('fumes_cover_monitoring_node')
    parser = ArgumentParser(description='Monitors the state of the fumes cover in the smelter')
    parser.add_argument('--mock', action='store_true', help='Loads a default image to test algorithm')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    monitor = FumesCoverMonitor(args.mock)
    if not args.mock:
        rospy.spin()
    else:
        pkg_name = RosPack().get_path('cascade_state_monitoring')
        img_off_fn = os.path.join(pkg_name, 'media', 'fumes_cover_off.png')
        img_off = cv2.imread(img_off_fn, cv2.IMREAD_GRAYSCALE)
        cover_off = monitor.is_fumes_cover_on(img_off)
        img_on_fn = os.path.join(pkg_name, 'media', 'fumes_cover_on.png')
        img_on = cv2.imread(img_on_fn, cv2.IMREAD_GRAYSCALE)
        cover_on = monitor.is_fumes_cover_on(img_on)
        stitch = cv2.hconcat((img_on, img_off))
        cv2.imshow('Fumes cover on (left) and Fumes cover off (right)', stitch)
        cv2.waitKey(0)
