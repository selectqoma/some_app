#!/usr/bin/env python3

import rospy
import cv2
from random import randint
from itertools import product
from slagsquare_groundplan_visualization.groundplan_visualization import GroundplanVisualizer
from smelter_msgs.msg import PotState, PotStateArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Temperature
import time

if __name__ == '__main__':
    viz = GroundplanVisualizer(ros=False)

    # create random pot states
    states = PotStateArray()
    states.header.stamp = rospy.Time(time.time())
    rows, cols = 8, 28
    ogm = OccupancyGrid()
    ogm.info.height = rows
    ogm.info.width = cols
    ogm.data = [0] * rows * cols
    for row, col in product(range(rows), range(cols)):
        # add pot to ogm
        prob = randint(0, 100)
        ogm.data[row * cols + col] = prob

        # add pot to pot state array
        state = PotState()
        state.position.y = row
        state.position.x = col
        t = Temperature(temperature=randint(20, 1000))
        state.temperature_history.append(t)
        states.pot_states.append(state)

    viz.states_msg_to_map(states, ogm)

    gtmpl = viz.draw_groundplan()
    cv2.imshow('Slagsquare Groundplan Template', gtmpl)
    cv2.waitKey(0)
