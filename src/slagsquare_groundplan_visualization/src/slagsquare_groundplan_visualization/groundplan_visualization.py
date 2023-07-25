""" groundplan_visualization - This module contains a class responsible for
    creating and publishing the slagsquare groundplan visualization
"""

import cv2
import numpy as np
import time
import datetime
from threading import Lock
from subprocess import check_output
from itertools import product

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import message_filters as mf
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

from smelter_msgs.msg import PotState, PotStateArray
from .groundplan_drawing_utils import draw_groundplan


class GroundplanVisualizer:
    """ A server that subscribes to pot state messages and creates a
        visualization which it publishes as an ROS image message
    """

    def __init__(self, ros=True):
        """ Initializes groundplan visualization parameters and creates
            subscribers and publihers

            Keyword arguments:
            ros -- Flag that determines whether to use ros interfaces
            (mostly for testing purposes)
        """
        self.load_params(ros)
        self.state_map = {}
        self.latest_timestamp = time.time()
        self.pot_states_lock = Lock()
        self.cv_bridge = CvBridge()

        # initial draw to publish an epmty plan before first update
        self.groundplan_template = self.draw_groundplan()

        # create subs, pubs, and timers if in ros mode
        if ros:

            # create subscribers to pots states and ogm
            pot_states_sub = mf.Subscriber('pot_states', PotStateArray)
            ogm_sub = mf.Subscriber('occupancy_grid/map', OccupancyGrid)

            # create synchronizer
            self.ts = mf.TimeSynchronizer([pot_states_sub, ogm_sub], 10)

            # register callback to synchronizer
            self.ts.registerCallback(self.pots_callback)

            # create dispaly publisher and timer that calls it
            self.groundplan_pub = rospy.Publisher('groundplan/image_raw', Image, queue_size=1)
            self.pub_timer = rospy.Timer(rospy.Duration(1.0/10.0), self.groundplan_pub_callback)

    def load_params(self, ros=True):
        """ Loads or initializes parameters

            Keyword arguments:
            ros -- Flag that determines whether params are loaded from ros
            param server or set to default values
        """
        if ros:
            self.width = rospy.get_param('~width')
            self.height = rospy.get_param('~height')
            self.grid_rows = rospy.get_param('~grid_rows')
            self.grid_cols = rospy.get_param('~grid_cols')
            self.presense_probability_threshold = rospy.get_param(
                '~presense_probability_threshold')
        else:
            self.width = 960
            self.height = 640
            self.grid_rows = 8
            self.grid_cols = 28
            self.presense_probability_threshold = 0.3

        self.pkg_path = rospkg.RosPack().get_path(
            'slagsquare_groundplan_visualization')
        try:
            self.git_version = check_output([
                'git',
                f'--git-dir={self.pkg_path}/../../.git',
                'describe',
                '--dirty',
                '--broken',
                '--tags']).decode('ascii').strip()
            self.git_hash = check_output([
                'git',
                f'--git-dir={self.pkg_path}/../../.git',
                'rev-parse',
                '--verify',
                'HEAD']).decode('ascii').strip()
        except Exception as e:
            self.git_version = "-- Version not found --"
            self.git_hash = "-- Version hash code not found --"
            rospy.logwarn('No tag found in this commit. A new version should '
                          'not be deployed without a tag.')


    def pots_callback(self, pots_msg, ogm_msg):
        """ Callback that receives pot states

            Keyword arguments:
            msg -- The pot states message for updating the visualization
        """
        rospy.logwarn('Updating groundplan visualization')

        with self.pot_states_lock:
            self.states_msg_to_map(pots_msg, ogm_msg)

    def states_msg_to_map(self, pots_msg, ogm_msg):
        """ Converts a pot states message to a dictionary

            Keyword arguments:
            msg -- The message to convert to a dictionary
        """
        self.latest_timestamp = pots_msg.header.stamp.to_sec()

        # iterate through the states and update temperatures
        for state in pots_msg.pot_states:
            x = int(state.position.x)
            y = int(state.position.y)

            self.state_map[(x, y)] = {
                    'temperature': state.temperature_history[0].temperature,
                    'filling_level': state.filling_level,
                    'composition': state.composition
            }

        # remove pots with low presense probabilities
        rows = ogm_msg.info.height
        cols = ogm_msg.info.width
        x_range = range(cols)
        y_range = range(rows)

        for x, y in product(x_range, y_range):
            p = ogm_msg.data[y * cols + x] / 100.0;

            if p < self.presense_probability_threshold and \
                    (x, y) in self.state_map:
                del self.state_map[(x, y)]


    def groundplan_pub_callback(self, event):
        """ Callback that draws the slagsquare groundplan with the latest
            pot states and then publishes it as a ROS message

            Keyword arguments:
            event -- the event id of the callback
        """

        # draw groundplan visualization with latest states
        groundplan = self.draw_groundplan()

        # convert groundplan image to ros message
        groundplan_msg = self.cv_bridge.cv2_to_imgmsg(groundplan, encoding="bgr8")
        groundplan_msg.header.stamp = rospy.Time(self.latest_timestamp)

        # publish groundplan message
        self.groundplan_pub.publish(groundplan_msg)

    def draw_groundplan(self):
        """ Wraps the draw_groundlan function for ease of use
        """
        #  copy states with atomic lock to prevent update during drawing
        with self.pot_states_lock:
            states = self.state_map.copy()

        # draw and return groundplan
        groundplan = draw_groundplan(
            shape=(self.height, self.width, 3),
            grid_shape=(self.grid_rows, self.grid_cols),
            states=states,
            stamp=self.latest_timestamp,
            git_version=self.git_version,
            git_hash=self.git_hash,
            warning='UNDER TEST')

        return groundplan
