import numpy as np
import zeep
from onvif import ONVIFCamera, ONVIFError
from ip_camera_driver.mock_onvif_camera import MockONVIFCamera
from datetime import datetime
import time

import rospy
from std_msgs.msg import Time
from sensor_msgs.msg import JointState
from smelter_msgs.msg import PanTiltZoom
import actionlib
from smelter_actions.msg import PTZAction, PTZFeedback, PTZResult

def zeep_pythonvalue(self, xmlvalue):
    return xmlvalue

zeep.xsd.simple.AnySimpleType.pythonvalue = zeep_pythonvalue

class ONVIFPTZControlDriver:

    def __init__(self):
        self.load_params()
        self.stamp = Time()
        self.ptz = None
        self.token = None
        self.ptz_msg = PanTiltZoom()
        self.ptz_msg.pan = self.position['pan']
        self.ptz_msg.tilt = self.position['tilt']
        self.joint_states = JointState()
        self.joint_states.header.frame_id = self.camera_name

        # create ptz data capturing object
        if not self.open_ptz_capture():
            exit(0)

        # initialize publishers and subscribers
        self.joint_states.name = [self.camera_name+'/pan_joint', self.camera_name+'/tilt_joint']
        self.joint_states.position = [self.position['pan'], self.position['tilt']]
        self.cmd_sub = rospy.Subscriber('~ptz/cmd', PanTiltZoom, self.set_ptz_callback, queue_size=10)
        self.ptz_pub = rospy.Publisher('~ptz/state', PanTiltZoom, queue_size=10)
        self.joint_states_pub = rospy.Publisher('~ptz/joint_states', JointState, queue_size=10)
        self.read_timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.read_ptz_callback)

        # initialize action server for ptz requests
        self.ptz_sas = actionlib.SimpleActionServer('ptz_cmd', PTZAction,
                execute_cb=self.ptz_action_callback, auto_start=False)
        self.ptz_sas.start()

    def load_params(self):
        self.camera_name = rospy.get_param('~camera_name')
        self.frame_id = rospy.get_param('~frame_id')
        self.ip = rospy.get_param('~uri/ip')
        self.user = rospy.get_param('~uri/username')
        self.password = rospy.get_param('~uri/password')
        self.port = rospy.get_param('~uri/port')
        self.uri_suffix = rospy.get_param('~uri/suffix')
        self.device_index = rospy.get_param('~uri/device_index')
        self.rate = rospy.get_param('~settings/ptz_rate')
        self.ptz_tolerance = rospy.get_param('~settings/ptz_tolerance')
        self.speed = rospy.get_param('~settings/default/speed')
        self.position = rospy.get_param('~settings/default/position')
        self.mock = rospy.get_param('/mock', False)

    def ptz_action_callback(self, goal):
        # write goal command
        ptz_feedback = None
        rospy.loginfo('Writing ptz command')
        start_time = rospy.Time.now()
        self.write_ptz_cmd(goal.ptz)
        rospy.loginfo('ptz command written in %f seconds'%(rospy.Time.now()-start_time).to_sec())

        # read the ptz state and provide feedback and finally the result
        r = rospy.Rate(10)
        success = False
        start_time = rospy.Time.now()
        while not success:
            if (rospy.Time.now()-start_time).to_sec() > 10.0:
                break
            #  if self.ptz_sas.is_preempt_requested():
                #  rospy.logwarn('ptz action preempted')
                #  self.ptz_sas.set_preempted()
                #  success = False
                #  break

            status = self.read_ptz()
            if status is None:
                break

            ptz_feedback = self.ptz_status_to_msg(status)
            c1 = abs(ptz_feedback.pan - goal.ptz.pan) < self.ptz_tolerance
            c2 = abs(ptz_feedback.tilt - goal.ptz.tilt) < self.ptz_tolerance
            c3 = abs(ptz_feedback.zoom - goal.ptz.zoom) < self.ptz_tolerance
            if c1 and c2 and c3:
                success = True
                break

            # publish feedback
            f = PTZFeedback()
            f.ptz = ptz_feedback
            f.elapsed_time = ptz_feedback.header.stamp - goal.ptz.header.stamp
            self.ptz_sas.publish_feedback(f)

        # publish result if succeeded
        if success:
            r = PTZResult()
            r.result = True
            r.ptz = ptz_feedback
            r.elapsed_time = ptz_feedback.header.stamp - goal.ptz.header.stamp
            self.ptz_sas.set_succeeded(r)
            rospy.loginfo('PTZ command executed successfully')
        else:
            self.ptz_sas.set_aborted()
            rospy.logwarn('PTZ command failed. Current state is: {}'.format(ptz_feedback))

    def open_ptz_capture(self):
        cam = None
        if self.mock:
            cam = MockONVIFCamera()
        else:
            try:
                cam = ONVIFCamera(self.ip, self.port, self.user, self.password, '/etc/onvif/wsdl')
            except ONVIFError:
                rospy.logerr('Failed to connect to camera for ptz control via the ONVIF protocol')
                return False
        try:
            media_service = cam.create_media_service()
            media_profile = media_service.GetProfiles()[self.device_index]
            self.token = media_profile._token
            self.ptz = cam.create_ptz_service()
        except Exception as e:
            rospy.logerr('Failed to initialize ONVIF camera ptz driver with error: {}'.format(e))
            return False
        return True

    def read_ptz_callback(self, timer):
        if self.ptz is None:
            rospy.logerr('read_ptz_callback() called before open_ptz_capture')
            return
        status = self.read_ptz()
        if status is None:
            rospy.logdebug('Failed to read PTZ state of camera. Device must be in use.')
            return
        # publish ptz states
        ptz_msg = self.ptz_status_to_msg(status)
        joint_states = self.ptz_status_to_joint_msg(status)
        self.ptz_pub.publish(ptz_msg)
        self.joint_states_pub.publish(joint_states)

    def set_ptz_callback(self, data):
        if data.home:
            self.ptz.GotoHomePosition({'ProfileToken': self.token})
        else:
            self.write_ptz_cmd(data)

    def write_ptz_cmd(self, data):
        cmd = self.ptz_msg_to_cmd(data)
        self.ptz.AbsoluteMove(cmd)

    def ptz_msg_to_cmd(self, data):
        cmd = self.ptz.create_type('AbsoluteMove')
        cmd.ProfileToken = self.token
        cmd.Position = {'PanTilt': {'_x': data.pan, '_y': data.tilt}, 'Zoom': {'_x': data.zoom}}
        cmd.Speed = {'PanTilt': {'_x': self.speed['pan'], '_y': self.speed['tilt']}, 'Zoom': {'_x': self.speed['zoom']}}
        return cmd

    def ptz_status_to_msg(self, status):
        ptz_msg = PanTiltZoom()
        ptz_msg.header.stamp = rospy.Time.now()#rospy.Time(datetime.timestamp(status['UtcTime']))
        ptz_msg.header.frame_id = self.frame_id
        ptz_msg.pan = status['Position']['PanTilt']['_x']
        ptz_msg.tilt = status['Position']['PanTilt']['_y']
        ptz_msg.zoom = status['Position']['Zoom']['_x']
        ptz_msg.home = False
        return ptz_msg

    def ptz_status_to_joint_msg(self, status):
        joint_states = JointState()
        joint_states.header.stamp = self.ptz_msg.header.stamp
        joint_states.position = [status['Position']['PanTilt']['_x'], status['Position']['PanTilt']['_y']]
        return joint_states

    def read_ptz(self):
        status = None
        try:
            status = self.ptz.GetStatus({'ProfileToken': self.token})
        except ONVIFError as e:
            rospy.logerr('Failed to get status with error {}'.format(e))
        return status
