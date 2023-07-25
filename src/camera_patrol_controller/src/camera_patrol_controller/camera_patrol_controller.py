import rospy
from smelter_msgs.msg import PanTiltZoom
from itertools import cycle, chain
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
import actionlib
from actionlib_msgs.msg import GoalStatus
from smelter_actions.msg import MoveAction, MoveGoal, MoveFeedback, MoveResult
from smelter_actions.msg import PTZAction, PTZGoal, PTZFeedback, PTZResult


class CameraPatrolController:

    def __init__(self):
        self.load_params()
        reverse_presets = self.ptz_presets[::-1]
        #  self.presets_pool = cycle(chain(self.ptz_presets, reverse_presets))
        self.presets_pool = cycle(self.ptz_presets)
        self.cmd_pub = rospy.Publisher('ptz/cmd', PanTiltZoom, queue_size=1)
        self.moved = False
        self.last_preset = None

        if self.update_mode == 'auto':
            self.timer = rospy.Timer(rospy.Duration(1.0/self.rate),
                                     self.cmd_pub_callback)
        elif self.update_mode == 'manual':
            self.sub = rospy.Subscriber(
                'move_camera', EmptyMsg, self.cmd_pub_callback)
            self.srv_server = rospy.Service(
                'move_camera', EmptySrv, self.cmd_pub_callback)
            self.move_sas = actionlib.SimpleActionServer(
                'move_camera', MoveAction,
                execute_cb=self.move_action_callback, auto_start=False)
            self.move_sac = actionlib.SimpleActionClient('ptz/cmd', PTZAction)
            self.sac_connected = False
            self.move_sas.start()
        else:
            error = 'ERROR: Parameter update_mode should be auto or manual'
            rospy.signal_shutdown(error)

    def load_params(self):
        self.update_mode = rospy.get_param('~update_mode', 'auto')
        self.ptz_presets = self.load_param('~ptz_presets')
        self.rate = rospy.get_param('~rate', 1)
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')

    def load_param(self, param):
        if rospy.has_param(param):
            value = rospy.get_param(param)
        else:
            error = f'Failed to load {param} from parameter server'
            rospy.signal_shutdown(error)
        return value

    def cmd_pub_callback(self, event):
        cmd = next(self.presets_pool)
        cmd_msg = self.cmd_to_msg(cmd)
        self.cmd_pub.publish(cmd_msg)

    def cmd_to_msg(self, cmd):
        cmd_msg = PanTiltZoom()
        cmd_msg.header.frame_id = self.frame_id
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.pan = cmd['pan']
        cmd_msg.tilt = cmd['tilt']
        cmd_msg.ir_zoom = cmd['zoom']
        return cmd_msg

    def move_action_callback(self, goal):
        if not self.sac_connected:
            self.sac_connected = \
                self.move_sac.wait_for_server(rospy.Duration(10.0))
            if not self.sac_connected:
                rospy.logerr('Could not connect to ptz/cmd action server. '
                             'Aborting move_camera goal request.')
                self.move_sas.set_aborted()
                return
        preset = None
        if goal.goal == MoveGoal.MOVE_HOME:
            preset = {'pan': 0.0, 'tilt': 0.0, 'zoom': 0.0}
        elif goal.goal == MoveGoal.MOVE_NEXT:
            preset = next(self.presets_pool)
        elif goal.goal == MoveGoal.MOVE_PREVIOUS:
            rospy.logwarn(
                'Previous camera pose move has not been implemented. '
                          'Moving to next pose instead.')
            preset = next(self.presets_pool)
        elif goal.goal == MoveGoal.MOVE_FIRST:
            preset = self.ptz_presets[0]
        elif goal.goal == MoveGoal.MOVE_LAST:
            preset = self.ptz_presets[-1]
        elif goal.goal == MoveGoal.MOVE_SAME:
            if self.last_preset is not None:
                preset = self.last_preset
            else:
                preset = next(self.presets_pool)
        else:
            rospy.logerr('Invalid camera goal pose requested')
            self.move_sas.set_aborted()
            return
        self.last_preset = preset
        ptz_goal = PTZGoal()
        ptz_goal.ptz = self.cmd_to_msg(preset)
        ptz_goal.ptz.home = (goal.goal == MoveGoal.MOVE_HOME)
        g = ptz_goal.ptz
        rospy.loginfo('Requesting camera to move to goal pose: {}'.format(
            self.get_ptz_msg_info(ptz_goal.ptz)))

        # request the camera to move using an action client that receives feedback
        self.move_sac.send_goal(ptz_goal, feedback_cb=self.move_feedback_callback)
        self.move_sac.wait_for_result(rospy.Duration(30.0))
        goal_state = self.move_sac.get_state()
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 30.0 and \
                goal_state == GoalStatus.ACTIVE:
            rospy.sleep(rospy.Duration(0.5))
            goal_state = self.move_sac.get_state()
            #  rospy.logwarn([(rospy.Time.now() - start_time).to_sec(), goal_state])
        if goal_state == GoalStatus.SUCCEEDED:
            r = self.move_sac.get_result()
            self.move_sas.set_succeeded(r)
        else:
            self.move_sas.set_aborted()

    def move_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.ptz
        move_feedback_msg = MoveFeedback()
        move_feedback_msg.ptz = feedback_msg.ptz
        move_feedback_msg.elapsed_time = feedback_msg.elapsed_time
        self.move_sas.publish_feedback(move_feedback_msg)

    def get_ptz_msg_info(self, msg):
        return {'pan': msg.pan, 'tilt': msg.tilt, 'ir_zoom': msg.ir_zoom} # , 'vis_zoom': msg.vis_zoom}
