""" retry_state.py - Contains the RetryState class that inherits from
    smach.State and implements a state with a maximum number of retries
"""

import rospy
import threading
import smach

class RetryState(smach.State):
    """ Class that implements a state with a maximum number of retries
    """

    def __init__(self, num_retries=0):
        """ Initializes the state

            Keyword arguments:
            num_retries -- The maximum number of retries in case of failure
        """
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['reset_counter'])
        self.num_retries = num_retries
        self.counter = 0

    def execute(self, userdata):
        """ The main routine of the RetryState

            Keyword arguments:
            userdata -- Contains the reset_counter
        """
        if userdata.reset_counter == True:
            self.counter = 0
        if self.counter != 0:
            rospy.logdebug('Retries {}/{}'.format(
                self.counter+1, self.num_retries))
        if self.counter < self.num_retries:
            self.counter += 1
            return 'succeeded'
        else:
            return 'aborted'
