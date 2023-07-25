''' slagsquare_monitoring_fsm.py - Contains the  SlagsquareMonitoringFSM class,
which implements a Finite State Machine that controls the transitions of the
slagsquare monitoring application
'''

import threading
import rospy
import smach
from smach_ros import IntrospectionServer, SimpleActionState, MonitorState
from std_msgs.msg import Empty
from smelter_actions.msg import MoveAction, MoveGoal
from smelter_actions.msg import TakeSnapshotAction
from smelter_actions.msg import UpdatePotStatesAction
from smelter_actions.msg import DetectObjectsAction

from .timeout_monitor_state import TimeoutMonitorState
from .retry_state import RetryState


class SlagsquareMonitoringFSM:
    """ Implements a Finite State Machine that controls the transitions of the
        slagsquare monitoring application
    """

    def __init__(self):
        """ Loads ros params and generates fsm graph
        """
        self.step_mode = rospy.get_param('~step_mode', False)
        self.viewer = rospy.get_param('~viewer', False)
        self.camera = rospy.get_param('~camera', 'camera')
        self.sis = None  # Introspection Server
        self.generate_fsm_graph()


    def generate_fsm_graph(self):
        """ Generates an FSM graph using the smach library
        """
        rospy.loginfo('Generating FSM graph')
        self.sm_root = smach.StateMachine(
            ['succeeded', 'aborted', 'preempted'])

        with self.sm_root:

            if self.step_mode:
                next_state = 'WAIT_FOR_STEP_COMMAND'
            else:
                next_state = 'MOVE'

            smach.StateMachine.add(
                'WAIT_FOR_INIT',
                MonitorState('~init', Empty, lambda arg1, arg2: False),
                transitions={'valid': 'WAIT_FOR_INIT', 'invalid': next_state})

            smach.StateMachine.add(
                'WAIT_FOR_STEP_COMMAND',
                MonitorState('~step_command', Empty, lambda arg1, arg2: False),
                transitions={'valid': 'aborted', 'invalid': 'MOVE'})

            smach.StateMachine.add(
                'MOVE',
                SimpleActionState(self.camera+'/move_camera', MoveAction,
                                  goal=MoveGoal(goal=MoveGoal.MOVE_NEXT),
                                  result_slots=['result', 'ptz']),
                transitions={'succeeded': 'TAKE_SNAPSHOTS',
                             'aborted': 'REATTEMPT_MOVE'},
                remapping={'result': 'reset_counter', 'ptz': 'camera_pose'})

            smach.StateMachine.add(
                'REATTEMPT_MOVE',
                SimpleActionState(
                    self.camera+'/move_camera', MoveAction,
                    goal=MoveGoal(goal=MoveGoal.MOVE_SAME),
                    result_slots=['result', 'ptz']),
                transitions={'succeeded': 'TAKE_SNAPSHOTS',
                             'aborted': 'REATTEMPT_MOVE'},
                remapping={'result': 'reset_counter', 'ptz': 'camera_pose'})

            smach.StateMachine.add(
                'TAKE_SNAPSHOTS', RetryState(num_retries=3),
                transitions={'succeeded': 'TAKE_CONCURRENT_SNAPSHOTS',
                             'aborted': 'REATTEMPT_MOVE'})

            sm_snapshots = smach.Concurrence(
                outcomes=['succeeded', 'preempted', 'aborted'],
                default_outcome='aborted',
                outcome_map={'succeeded': {'TAKE_THERMAL_SNAPSHOT':'succeeded',
                                           'VISUAL_DETECTIONS':'succeeded'}},
                input_keys=['camera_pose'],
                output_keys=['visual_snapshot', 'thermal_snapshot',
                             'detected_pots'])

            smach.StateMachine.add(
                'TAKE_CONCURRENT_SNAPSHOTS', sm_snapshots,
                transitions={'succeeded': 'UPDATE_POT_STATES',
                             'aborted': 'TAKE_SNAPSHOTS'})

            with sm_snapshots:

                smach.Concurrence.add(
                    'TAKE_THERMAL_SNAPSHOT',
                    SimpleActionState(
                        self.camera+'/ir/take_snapshot',
                        TakeSnapshotAction,
                        goal_slots=['ptz'],
                        result_slots=['success', 'snapshot']),
                    remapping={'ptz': 'camera_pose',
                               'snapshot': 'thermal_snapshot'})

                sm_visual_detections = smach.StateMachine(
                    outcomes=['succeeded','preempted','aborted'],
                    input_keys=['camera_pose'],
                    output_keys=['detected_pots', 'visual_snapshot',
                                 'reset_counter'])

                with sm_visual_detections:
                    smach.StateMachine.add(
                        'TAKE_VISUAL_SNAPSHOT',
                        SimpleActionState(
                            self.camera+'/rgb/take_snapshot',
                            TakeSnapshotAction,
                            goal_slots=['ptz'],
                            result_slots=['success', 'snapshot']),
                        transitions={'succeeded': 'DETECT_POTS'},
                        remapping={'ptz': 'camera_pose',
                                   'snapshot': 'visual_snapshot'})

                    smach.StateMachine.add(
                        'DETECT_POTS',
                        SimpleActionState(
                            self.camera+'/rgb/detect_pots', DetectObjectsAction,
                            goal_slots=['image'],
                            result_slots=['success', 'detected_objects']),
                        remapping={'success': 'reset_counter',
                                   'image': 'visual_snapshot',
                                   'detected_objects': 'detected_pots'})

                smach.Concurrence.add('VISUAL_DETECTIONS',
                                      sm_visual_detections)

            smach.StateMachine.add(
                'UPDATE_POT_STATES',
                SimpleActionState(
                    'update_pot_states', UpdatePotStatesAction,
                    goal_slots=['thermal_snapshot', 'detected_pots',
                                'camera_pose'],
                    result_slots=['result']),
                transitions={'succeeded': next_state,
                             'aborted': 'REATTEMPT_MOVE'},
                remapping={'result': 'reset_counter'})


    def execute(self):
        """ Starts the FSM
        """
        self.sm_thread = threading.Thread(target=self.sm_root.execute)
        self.sm_thread.start()


    def spawn_introspection_server(self):
        """ Starts the introspection server for the visualization of the FSM
        """
        if not self.viewer:
            return
        self.sis = IntrospectionServer('~introspection_server',
                                       self.sm_root, '/SM_ROOT')
        self.sis.start()


    def stop_introspection_server(self):
        """ Stops the introspection server for the visualization of the FSM
        """
        if not self.viewer:
            return

        if self.sis is None:
            rospy.warn(
                'Introspection Server already stopped or it was never started')

        self.sis.stop()
        self.sis = None

