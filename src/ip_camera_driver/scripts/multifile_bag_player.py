#!/usr/bin/env python3

import os
import sys
import time
import getopt
import rosbag
import rospy
import subprocess
from pprint import pprint
from math import ceil

def usage():
    print( 'Usage: -p PATH -r RATE -n NUM\n')
    print('-h, --help        Display this help and exit')
    print('-p, --path=PATH   Specify directory of rosbags. If none is provided a default one will be used.')
    print('-r, --rate=RATE   Select rate for playback')
    print('-n, --num=NUM     Specify number of bags to load per iteration')
    print('-s, --start=START Specify from which file to start')
    print('-e, --end=END     Specify on which file to end\n')
    print('-t, --step=STEP   Specify stride for file skipping\n')

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(rospy.myargv()[1:], 'hp:r:n:s:e:t:', ['help', 'path=', 'rate=', 'num=', 'start=', 'end=', 'step='])
        path = '/media/'+os.environ['USER']+'/Elements/george/umicore/cascade/20200205/bags/1'
        rate, num, start, end, step = 1, 1, 1, None, 1
        for opt, arg in opts:
            if opt in ('-h', '--help'):
                #  print(opt)
                usage()
                sys.exit()
            elif opt in ('-p', '--path') or '-p' in opt and arg:
                path = arg
            elif opt in ('-r', '--rate') or '-r' in opt and arg:
                rate = int(arg)
            elif opt in ('-n', '--num') or '-n' in opt and arg:
                num = int(arg)
            elif opt in ('-s', '--start') or '-s' in opt and arg:
                start = int(arg)
            elif opt in ('-e', '--end') or '-e' in opt and arg:
                end = int(arg)
            elif opt in ('-t', '--step') or '-t' in opt and arg:
                step = int(arg)
            else:
                print('Unknown option ' + opt)
                print('')
                usage()
                sys.exit(2)
    except (ValueError, getopt.GetoptError):
        usage()
        sys.exit(2)

    files = sorted(os.listdir(path))
    if end is None:
        end = len(files)
    files = files[start:end:step]
    file_lists = [files[i*num:i*num+num] for i in range(ceil(len(files)/num))]

    for i, bagfiles in enumerate(file_lists):
        filepaths = [os.path.join(path, f) for f in bagfiles]
        print(f'Playing {i*num}-{(i+1)*num} out of {len(files)} bagfiles. Progress: {int((i+1)/len(file_lists)*100)}%')
        player = subprocess.Popen(['rosbag', 'play', '-r'+str(rate), *filepaths], stdout=subprocess.PIPE)
        try:
            player.communicate()[0]
        except KeyboardInterrupt:
            break
        finally:
            del player
