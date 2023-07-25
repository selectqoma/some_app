""" utils.py -- This file contains utility functions for the pot
    state server package
"""

import os
import time
import pickle
from itertools import product
import numpy as np

import rospy

from smelter_msgs.msg import Contour

from .grid_model import GridModel


def contour_msg_to_array(contour_msg: Contour) -> np.array:
    """ Convert a contour array ROS message to a numpy array

    Args:
        contour(): The contour array message to convert to a numpy array

    Returns:
        np.array: The contour array in numpy array format

    Raises:
        ValueError
    """
    if not isinstance(contour_msg, Contour):
        raise ValueError(
            f'contour_msg is of type {type(contour_msg)} instead '
            f'of expected type smelter_msgs.msg.ContourArray')

    points = []
    for point in contour_msg.points:
        points += [[point.x, point.y]]

    contour = np.array(points).astype(np.int)

    return contour


def get_approximate_poses(pose: tuple, precision: int = 2) -> list:
    """ Returns list with 5 poses rounded up and down of original pose

    Args:
        pose(tuple) -- The camera pan-tilt-zoom pose
        precision(float) -- Number of decimal points

    Returns:
        list: A list of approximate poses around the given pose

    Raises:
        ValueError: If pose argument is not of type tuple
        ValueError: If pose argument has less or more than 3 elements
        ValueError: If precision type is not int
        ValueError: If precision is lower or equal to zero
    """
    if not isinstance(pose, tuple):
        raise ValueError(f'The pose argument if of type {type(pose)}. '
                         'Expected argument of type tuple.')

    if len(pose) != 3:
        raise ValueError('The pose argument is of type tuple with {len(pose)} '
                         f'elements, but expected a tuple of 3 elements.')

    if not isinstance(precision, int):
        raise ValueError('The precision argument is of type '
                         f'{type(precision)}, but expected type int.')

    if precision <= 0:
        raise ValueError('The precision argument is lower or equal to zero')

    poses = [pose]
    inc = 0.5 / 10 ** precision
    for dpan, dtilt in product([-inc, inc], [-inc, inc]):
        poses += [tuple([round(pose[0] + dpan, precision),
                         round(pose[1] + dtilt, precision),
                         round(pose[2], precision)])]

    return poses


def is_close_enough(list1: list, list2: list, epsilon:float = 1e-2):
    """ Returns true if the errors between two lists of values
        are small enough

    Args:
        list1(list): The first list of values
        list2(list): The second list of values
        epsilon(float): The threshold for the error

    Returns:
        bool: True if the errors between the list elements are small enough

    Raises:
        ValueError: If argument list1 is not of type list
        ValueError: If argument list2 is not of type list
        ValueError: If arguments list1 and list2 have different size
    """
    if not isinstance(list1, list):
        raise ValueError(f'Argument list1 if of type {type(list1)}, but '
                         'expected argument of type list')

    if not isinstance(list2, list):
        raise ValueError(f'Argument list2 if of type {type(list2)}, but'
                         'expected argument of type list')

    errors = np.array(list1) - np.array(list2)

    return all(np.abs(errors) < epsilon)


def save_latest_grid_state(grid: GridModel) -> bool:
    """ Saves the latest grid state for recovery in case of failure or restart

    Args:
        grid(GridModel): The GridModel to pickle and save to file

    Returns:
        bool: True if save was succesful

    Raises:
        ValueError: If argument grid is not of type GridModel
    """
    if not isinstance(grid, GridModel):
        raise ValueError('Argument grid if of type {type(grid)}, but '
                         'expected argument of type GridModel')

    path = os.path.join(os.path.expanduser('~'), '.ros',
                        'slagsquare_grid_states.pickle')

    with open(path, 'wb') as out:

        try:
            pickle.dump(grid, out)
            rospy.loginfo(f'Saved latest grid state')

            return True
        except pickle.PicklingError as exc:
            rospy.logerr(f'Failed to save grid state with error {exc}')

        return False


def load_latest_grid_state(expiration_time: int = 30):
    """ Loads the latest grid state not older than the expiration_time

    Args:
        expiration_time(int): After how much time a pickled grid model
                              expires

    Returns:
        bool: A flag indicating whether loading succeeded
        GridModel: The grid model that was loaded from the saved pikle
    """
    path = os.path.join(os.path.expanduser('~'), '.ros',
                        'slagsquare_grid_states.pkl')

    if os.path.exists(path):
        mod_time = os.path.getmtime(path)
        elapsed_time = time.time() - mod_time

        if elapsed_time < 60 * expiration_time: # < x minutes

            try:
                with open(path, 'rb') as file:
                    grid = pickle.load(file)
                rospy.logwarn('Loading grid states from {} minutes ago'
                              ''.format(round(elapsed_time/60, 2)))

                return True, grid
            except pickle.UnpicklingError as exc:
                rospy.logerr('Failed to load grid states from {} minutes '
                             'ago with error {}'.format(
                                 round(elapsed_time / 60, 2), exc))

                return False, None
        else:
            rospy.logwarn('Did not load latest grid states because they '
                          'are too old ({} minutes old)'
                          ''.format(elapsed_time / 60))

            return False, None
    else:
        rospy.logwarn('Did not find previous grid states file. '
                      'Starting from scratch.')

        return False, None
