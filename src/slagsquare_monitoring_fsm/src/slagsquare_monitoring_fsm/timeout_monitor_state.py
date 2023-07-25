import rospy
import threading
import smach

"""
A state that will wait for a message on a ROS topic or until a certain time has passed.
copied from:
    https://gist.github.com/mightyCelu/462cf6ff5e945dbc23739594dcd5ebd4
"""
class TimeoutMonitorState(smach.State):
    OUTCOME_RECEIVED_TRUE = 'update'
    OUTCOME_RECEIVED_FALSE = 'reset'
    OUTCOME_TIMEOUT = 'timeout'
    OUTCOME_PREEMPTED = 'preempted'

    """State Initializer
    @type topic string
    @param topic the topic to monitor
    @type msg_type a ROS message type
    @param msg_type determines the type of the monitored topic
    @type timeout rospy.Duration
    @param timeout the maximum time to wait for a message. In order to omit the timeout, pass None.
    @type count int
    @param count the number of messages to wait for.
    """
    def __init__(self, topic, msg_type, timeout, count=1, input_keys=[], output_keys=[]):
        smach.State.__init__(
            self,
            outcomes=[self.__class__.OUTCOME_RECEIVED_TRUE,
                      self.__class__.OUTCOME_RECEIVED_FALSE,
                      self.__class__.OUTCOME_TIMEOUT,
                      self.__class__.OUTCOME_PREEMPTED],
            input_keys=input_keys,
            output_keys=output_keys)
        self._topic = topic
        self._msg_type = msg_type
        self._timeout = timeout
        self._expected_message_count = count
        self._update = None
        self._trigger_event = threading.Event()

    def execute(self, ud):
        # If prempted before even getting a chance, give up.
        if self.preempt_requested():
            self.service_preempt()
            return self.__class__.OUTCOME_PREEMPTED

        self._received_message_count = 0
        self._trigger_event.clear()

        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._message_cb, callback_args=ud)
        # might want to add a timeout callback for additional functionality
        if self._timeout:
            self._timeout_timer = rospy.Timer(self._timeout, lambda _: self._trigger_event.set(), oneshot=True)

        self._trigger_event.wait()
        self._sub.unregister()
        if self._timeout:
            self._timeout_timer.shutdown()

        if self.preempt_requested():
            self.service_preempt()
            return self.__class__.OUTCOME_PREEMPTED

        if self._received_message_count != self._expected_message_count:
            return self.__class__.OUTCOME_TIMEOUT

        if self.update:
            return self.__class__.OUTCOME_RECEIVED_TRUE
        else:
            return self.__class__.OUTCOME_RECEIVED_FALSE

    def _message_cb(self, msg, ud):
        self._received_message_count += 1
        try:
            self.update = msg.data
        except AttributeError:
            self.update = True

        if self._received_message_count == self._expected_message_count:
            self._trigger_event.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()
