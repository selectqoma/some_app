#!/usr/bin/env python3

import os
import cv2
import sys
import time
import getopt
import numpy as np

import rospy
import rospkg
import rosparam
import rosbag
from cv_bridge import CvBridge

def usage():
    print( 'Usage: -p PATH -r RATE -n NUM\n')
    print('-h, --help             Display this help and exit')
    print('-f, --file=FILE        Specify rosbag path.')
    print('-i, --iter=ITERATION   Specify iteration to store.')
    print('\nResults will be stored in ~/.ros/calbration_frames')

def find_closest_pose(pose, poses):
    min_error = 1000
    min_index = -1
    for index, p in enumerate(poses):
        error = get_error(pose, p)
        if error < min_error:
            min_error = error
            min_index = index
    ret = min_index != -1
    return ret, min_index

def get_error(p1, p2):
    e_pan = p1['pan'] - p2['pan']
    e_tilt = p1['tilt'] - p2['tilt']
    e_zoom = p1['zoom'] - p2['zoom']
    return np.sqrt(e_pan**2 + e_tilt**2 + e_zoom**2)

if __name__ == '__main__':
    rospy.init_node('rosbag_image_extractor')
    try:
        opts, args = getopt.getopt(rospy.myargv()[1:], 'hf:i:', ['help', 'file=', 'iter='])
        filepath = None
        iteration = 0
        for opt, arg in opts:
            if opt in ('-h', '--help'):
                usage()
                sys.exit()
            elif opt in ('-f', '--file') or '-f' in opt and arg:
                filepath = arg
            elif opt in ('-i', '--iter') or '-i' in opt and arg:
                iteration = int(arg)
            else:
                print('Unknown option ' + opt)
                print('')
                usage()
                sys.exit(2)

        if filepath is None:
            rospy.logerr('Filepath not specified')
            usage()
            sys.exit(0)
    except (ValueError, getopt.GetoptError):
        usage()
        sys.exit(2)

    # create save locations
    out_path = os.path.join(os.path.expanduser('~'), '.ros', 'calibration_frames')
    if not os.path.isdir(out_path):
        os.mkdir(out_path)
    if not os.path.isdir(os.path.join(out_path, 'ir')):
        os.mkdir(os.path.join(out_path, 'ir'))
    if not os.path.isdir(os.path.join(out_path, 'rgb')):
        os.mkdir(os.path.join(out_path, 'rgb'))

    # load poses
    poses_pkg_path = rospkg.RosPack().get_path('camera_patrol_controller')
    poses_path = os.path.join(poses_pkg_path, 'config', 'camera_patrol_controller_params.yaml')
    poses = rosparam.load_file(poses_path)[0][0]['ptz_presets']

    topic_list = ['/slagsquare/flir_a310pt/rgb/image_raw',
                  '/slagsquare/flir_a310pt/ir/normalized/image_raw',
                  '/slagsquare/flir_a310pt/ptz/state']
    bag = rosbag.Bag(filepath)
    pose = None
    count = 0
    iteration_count = 0
    for topic, msg, t in bag.read_messages(topics=topic_list):
        if iteration_count < iteration * len(poses) * len(topic_list):
            iteration_count += 1
            continue

        if 'ptz/state' in topic:
            p = {'pan': msg.pan, 'tilt': msg.tilt, 'zoom': msg.zoom}
            ret, index = find_closest_pose(p, poses)
            if not ret:
                exit(0)
            else:
                pose = poses[index]
                count += 1
        elif 'rgb/image_raw' in topic:
            img = CvBridge().imgmsg_to_cv2(msg)
            fn = '{}_rgb_image_{}_{}_{}.png'.format(str(count).zfill(4), pose['pan'], pose['tilt'], pose['zoom'])
            cv2.imwrite(os.path.join(out_path, 'rgb', fn), img)
        elif 'ir/normalized/image_raw' in topic:
            img = CvBridge().imgmsg_to_cv2(msg)
            fn = '{}_ir_image_{}_{}_{}.png'.format(str(count).zfill(4), pose['pan'], pose['tilt'], pose['zoom'])
            cv2.imwrite(os.path.join(out_path, 'ir', fn), img)

            if count == len(poses):
                break
