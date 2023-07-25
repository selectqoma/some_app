""" flir_cgi_driver_ros.py - This module contains the FLIRCGIDRiverWrapper and
FLIRCGIDriverROS classes that wrap the FLIRCGIDRiver class and exposing
communication interfaces for controlling a camera and getting frames via
ROS topics
"""

import os
from threading import Lock
import wget
import cv2
import numpy as np

import rospy
import rospkg
from actionlib import SimpleActionServer
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from smelter_msgs.msg import PanTiltZoom
from smelter_srvs.srv import WritePTZ, WritePTZResponse
from smelter_actions.msg import TakeSnapshotAction, TakeSnapshotResult
from smelter_actions.msg import PTZAction, PTZResult

from ip_camera_driver.flir_cgi_driver import FlirCGIDriver


ENCODINGS = {'8UC1': 'mono8',
             '8UC3': 'bgr8',
             '16UC1': 'mono16',
             '16UC3': 'bgr16'}


def pose_to_dict(pan, tilt, ir_zoom, vis_zoom):
    """ Creates a dictionary with PTZ values

        Keyword arguments:
        pan -- The pan value in radians
        tilt -- The tilt value in radians
        ir_zoom -- The ir_zoom value in radians
        vis_zoom -- The vis_zoom value in radians
    """
    return {
        'pan': pan,
        'tilt': tilt,
        'ir_zoom': ir_zoom,
        'vis_zoom': vis_zoom
    }


class FlirCGIDriverWrapper:

    def __init__(self, ip, port, mock=False, camera_name="FLIR",
                 store_snapshots=False,
                 pt_offsets={'pan': 0, 'tilt': 0}):
        self.camera_name = camera_name
        self.ip = ip
        self.port = port
        self.mock = mock
        self.camera_pose = pose_to_dict(pan=0, tilt=0, ir_zoom=0, vis_zoom=8)

        if store_snapshots:
            self.fff_storage_path = os.path.join(
                os.path.expanduser('~'), '.ros', 'fff')
            if not os.path.exists(self.fff_storage_path):
                try:
                    os.mkdir(self.fff_storage_path)
                except Exception as e:
                    rospy.logerr(f'Failed to create fff storage path in '
                                 '{self.fff_storage_path} with exception {e}')
                    self.fff_storage_path = None
        else:
            self.fff_storage_path = None

        if self.mock:
            dataset_pkg = 'smelter_datasets_and_models'
            dataset_pkg_path = rospkg.RosPack().get_path(dataset_pkg)
            dir_seq = ['slagsquare_monitoring', 'patrol_frames', 'fff']
            self.dataset_path = os.path.join(dataset_pkg_path, *dir_seq)
            self.frame_filenames = os.listdir(self.dataset_path)
        else:
            # create flir cgi driver instance if not in mock mode
            self.drv = FlirCGIDriver(self.ip, self.port,
                                     fff_storage_path=self.fff_storage_path,
                                     pt_offsets=pt_offsets)


    def start_session(self, control=True, ping=True):
        return self.mock or self.drv.start_session(control, ping)


    def request_zoom_slave_write(self, ir, vis):
        return self.mock or self.drv.request_zoom_slave_write(ir, vis)


    def validate_zoom_slaving(self):
        if self.mock:
            return True

        states = self.drv.request_zoom_slave_read()
        return states['IRZoomSlave'] == 0 and states['DLTVZoomSlave'] == 1


    def read_snapshot(self, format_id=0, range_mode=0):
        if self.mock:
            return self.mock_read_snapshot()

        return self.drv.read_snapshot(format_id, range_mode)


    def mock_read_snapshot(self):
        filename = None
        pan = self.camera_pose['pan']
        tilt = self.camera_pose['tilt']
        ir_zoom = self.camera_pose['ir_zoom']
        for fn in self.frame_filenames:
            if "{}_{}_{}".format(pan, tilt, ir_zoom) in fn:
                filename = fn
                break
        else:
            return None, None

        path = os.path.join(self.dataset_path, filename)

        if self.fff_storage_path is not None:
            wget.download('file://'+path, out=self.fff_storage_path)
        rospy.loginfo('[{}] Reading frame:\n{}'.format(self.camera_name, path))
        metadata, frame = None, cv2.imread(path, cv2.IMREAD_UNCHANGED)
        # TODO replace above with below when fff images are in datasets
        #  metadata, frame = self.drv.mock_read_snapshot(f'file://{path}')
        return metadata, frame


    def read_ptz(self):
        if self.mock:
            return self.camera_pose

        return self.drv.read_ptz()


    def write_ptz(self, data):
        ''' Writes ptz command to camera

            The write_ptz function of the driver accepts 4 parameters namely
            pan, tilt, ir_zoom, vis_zoom. Each parameter has a default value
            of None. Since we provide three values to the function and the 4th
            is left as None, this command writes pan, tilt, and ir_zoom
        '''
        self.camera_pose = pose_to_dict(pan=data.pan, tilt=data.tilt,
                                        ir_zoom=data.ir_zoom,
                                        vis_zoom=data.vis_zoom)
        if self.mock:
            self.camera_pose['vis_zoom'] = 8.2
            return True

        return self.drv.write_ptz(pan=self.camera_pose['pan'],
                                  tilt=self.camera_pose['tilt'],
                                  ir_zoom=self.camera_pose['ir_zoom'])
                                  #  vis_zoom=self.camera_pose['vis_zoom'])


    def get_ping_status(self):
        if self.mock:
            return 'OK'
        else:
            if self.drv.ping_status:
                return 'OK'
            else:
                return 'Camera connection failure'


class FlirCGIDriverROS:

    def __init__(self):
        # initialize member variables
        self.cv_bridge = CvBridge()
        self.seq = 0
        self.frame = None
        self.load_params()
        self.drv_lock = Lock()

        # initialize diagnostics interface
        self.init_diagnostics()

        # initialize camera driver
        self.reset_driver()

        # init timers
        if self.ptz:
            self.init_ptz_interfaces()

        # init frame subs, servers
        self.init_frame_interfaces()


    def reset_driver(self):
        self.drv = FlirCGIDriverWrapper(self.ip, self.port, self.mock,
                                        self.camera_name,
                                        self.store_fff_snapshots,
                                        self.pt_offsets)
        ret = self.drv.start_session(control=True, ping=True)
        if ret is None or ret is False:
            rospy.logerr('Failed to properly start a camera CGI session.')
        elif not self.drv.request_zoom_slave_write(ir=False, vis=True):
            rospy.logerr('Failed to set zoom slaving on reset')


    def load_params(self):
        # load params from param server
        self.mock = rospy.get_param('~mock', False)
        self.camera_name = rospy.get_param('~camera_name')
        self.frame_id = rospy.get_param('~frame_id')
        self.ip = rospy.get_param('~uri/ip')
        self.port = rospy.get_param('~uri/cgi_port')
        self.framerate = rospy.get_param('~settings/framerate')
        self.manual_acq = rospy.get_param('~settings/manual_acquisition')
        self.ptz = rospy.get_param('~settings/ptz')
        self.store_fff_snapshots = rospy.get_param(
            '~settings/store_fff_snapshots', False)
        if self.ptz:
            self.ptz_action_timeout = rospy.get_param(
                '~settings/ptz_action_timeout')
            self.ptz_tolerance = rospy.get_param('~settings/ptz_tolerance')
            self.ptz_rate = rospy.get_param('~settings/ptz_rate')
            self.pt_offsets = rospy.get_param('~settings/pt_offsets')


    def init_diagnostics(self):
        self.ptz_status = 'N/A'
        self.snapshot_status = 'N/A'
        self.diagnostics_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=1)
        self.diagnostics_timer = \
            rospy.Timer(rospy.Duration(1.0), self.diagnostics_callback)


    def init_ptz_interfaces(self):
        self.camera_pose = None
        # initialize ptz state publisher
        self.ptz_pub = rospy.Publisher('ptz/state', PanTiltZoom, queue_size=10)

        # intialize ptz reading timer
        self.read_ptz_timer = rospy.Timer(
            rospy.Duration(1.0/self.ptz_rate), self.read_ptz_callback)

        # initialize ptz command subscriber, service, and action server
        self.ptz_sub = rospy.Subscriber(
            'ptz/cmd', PanTiltZoom, self.write_ptz_callback, queue_size=1)
        self.ptz_srv = rospy.Service(
            'ptz/cmd', WritePTZ, self.write_ptz_srv_callback)
        self.ptz_sas = SimpleActionServer(
            'ptz/cmd', PTZAction, execute_cb=self.write_ptz_action_callback,
            auto_start=False)
        self.ptz_sas.start()


    def init_frame_interfaces(self):
        # intialize frame publishers
        self.frame_pub = rospy.Publisher(
            'snapshot/image_raw', Image, queue_size=10)
        self.normalized_frame_pub = rospy.Publisher(
            'snapshot/normalized/image_raw', Image, queue_size=10)

        # initialize frame reading timer if auto mode is set
        if not self.manual_acq:
            self.read_timer = rospy.Timer(
                rospy.Duration(1.0/self.framerate), self.read_frame_callback)

        # initialize subscriber, service, and action server
        self.rs_sub = rospy.Subscriber(
            'take_snapshot', Empty, self.read_frame_callback)
        self.rs_srv = rospy.Service(
            'take_snapshot', GetPolledImage, self.read_frame_srv_callback)
        self.rs_sas = SimpleActionServer(
            'take_snapshot', TakeSnapshotAction,
            execute_cb=self.read_frame_action_callback, auto_start=False)
        self.rs_sas.start()


    def diagnostics_callback(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()

        status = DiagnosticStatus()
        status.name = self.camera_name
        status.level = DiagnosticStatus.ERROR

        ping_status = self.drv.get_ping_status()

        if self.ptz_status == 'OK' and self.snapshot_status == 'OK' and \
                ping_status == 'OK':
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        else:
            status_message = 'Failure: '
            for s in [self.ptz_status, self.snapshot_status, ping_status]:
                if s != 'OK':
                    status_message += f';{s}'

        status.values.append(KeyValue(key='ping_status',
                                      value=str(self.drv.get_ping_status())))
        status.values.append(KeyValue(key='ptz_status',
                                      value=self.ptz_status))
        status.values.append(KeyValue(key='snapshot_status',
                                      value=self.snapshot_status))

        msg.status.append(status)
        self.diagnostics_pub.publish(msg)

    def read_frame(self):
        for _ in range(3):
            _, frame = self.drv.read_snapshot(format_id=0, range_mode=2)
            if frame is None:
                rospy.logwarn('Failed to read frame')
                self.snapshot_status = 'Snapshot capture failure'
            elif self.frame is not None and frame == self.frame:
                rospy.logwarn('Snapshot update failure. Camera reboot '
                              'required')
                self.snapshot_status = 'Snapshot update failure ' + \
                    'camera reboot required.'
            else:
                self.snapshot_status = 'OK'
                break
        else:
            rospy.logerr('Failed to read snapshot after 3 tries')
            return None, None
        return frame, rospy.Time.now()


    def magnify_frame(self, frame, zoom):
        h, w = frame.shape[:2]
        cx = frame.shape[1] // 2
        cy = frame.shape[0] // 2
        radius_x = int(w * (1 - zoom / 100.0) / 2)
        radius_y = int(h * (1 - zoom / 100.0) / 2)
        min_x = cx - radius_x
        max_x = cx + radius_x
        min_y = cy - radius_y
        max_y = cy + radius_y
        crop = frame[min_y:max_y, min_x:max_x]
        resized_crop = cv2.resize(crop, (w, h))
        transformed_crop = self.transform_frame(resized_crop)
        return transformed_crop


    def transform_frame(self, frame):
        #  H = np.asarray([
            #  [1.9811061685238194, 0.04404042453085638, 36.443345268610734,],
            #  [0.004990683935938353, 2.0339065236282026, -7.344006052040411],
            #  [7.228009321082053e-06, 5.447765264982036e-05, 1.0]])
        #  warped_image = cv2.warpPerspective(frame, H, (704, 480))
        resized_frame = cv2.resize(frame, (630,470))
        transformed_frame = cv2.copyMakeBorder(resized_frame, 0, 10, 43, 31,
                                               cv2.BORDER_CONSTANT,
                                               value=(resized_frame.min(),)*4)
        return transformed_frame


    def read_frame_callback(self, data):
        frame, timestamp = self.read_frame()
        if frame is not None:
            self.publish_frame(frame, timestamp)


    def read_frame_srv_callback(self, req):
        frame, timestamp = self.read_frame()
        response = GetPolledImageResponse()
        response.stamp = timestamp
        if frame is not None:
            self.publish_frame(frame, timestamp)
            response.success = True
        else:
            response.success = False
        return response


    def read_frame_action_callback(self, goal):
        rospy.loginfo('Received action goal to take snapshot for camera '
                      f'{self.camera_name}')
        self.drv.camera_pose = pose_to_dict(pan=goal.ptz.pan,
                                            tilt=goal.ptz.tilt,
                                            ir_zoom=goal.ptz.ir_zoom,
                                            vis_zoom=goal.ptz.vis_zoom)
        frame, timestamp = self.read_frame()

        if frame is not None:
            r = TakeSnapshotResult()
            r.success = True
            r.snapshot = self.cv2_to_imgmsg(frame, timestamp)
            self.rs_sas.set_succeeded(r)
            self.publish_frame(frame, timestamp)
        else:
            rospy.logerr(
                f'Failed to read frame from camera {self.camera_name}')
            self.rs_sas.set_aborted()
            self.reset_driver()


    def publish_frame(self, frame, timestamp):
        frame_msg = self.cv2_to_imgmsg(frame, timestamp, False)
        normalized_frame_msg = self.cv2_to_imgmsg(frame, timestamp, True)
        if frame_msg is not None:
            self.frame_pub.publish(frame_msg)
        if normalized_frame_msg is not None:
            self.normalized_frame_pub.publish(normalized_frame_msg)


    def cv2_to_imgmsg(self, frame, timestamp, normalize=False):
        if normalize:
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX,
                                  cv2.CV_8U)
        try:
            frame_msg = self.cv_bridge.cv2_to_imgmsg(frame,
                                                     encoding="passthrough")
        except CvBridgeError as e:
            rospy.logwarn('Failed to convert cv image to ros message with '
                          'exception {}'.format(e))
            return None
        # publish frame
        frame_msg.header.stamp = timestamp
        frame_msg.header.seq = self.seq + 1; self.seq += 1
        frame_msg.header.frame_id = self.frame_id
        if frame_msg.encoding in ENCODINGS:
            frame_msg.encoding = ENCODINGS[frame_msg.encoding]
        return frame_msg


    def read_ptz(self):
        with self.drv_lock:
            timestamp = rospy.Time.now()
            ret = self.drv.read_ptz()

        if ret is None:
            self.ptz_status = "PTZ state reading failure"
        else:
            self.ptz_status = 'OK'

        return ret, timestamp


    def write_ptz(self, data):
        with self.drv_lock:
            timestamp = rospy.Time.now()
            ret = self.drv.write_ptz(data)

        if ret is None:
            self.ptz_status = "PTZ command execution failure"
        else:
            self.ptz_status = 'OK'

        return ret, timestamp


    def read_ptz_callback(self, event):
        ret, timestamp = self.read_ptz()
        if ret is None:
            return
        pan = ret['pan']
        tilt = ret['tilt']
        ir_zoom = ret['ir_zoom']
        vis_zoom = ret['vis_zoom']
        self.camera_pose = {
            'pan': pan,
            'tilt': tilt,
            'ir_zoom': ir_zoom,
            'vis_zoom': vis_zoom
        }
        ptz_msg = self.ptz_to_msg(pan, tilt, ir_zoom, vis_zoom, timestamp)
        self.ptz_pub.publish(ptz_msg)


    def write_ptz_callback(self, data):
        ret, _ = self.write_ptz(data)
        if not ret:
            rospy.logwarn('Failed to write PTZ cmd via message. Device busy.')


    def write_ptz_srv_callback(self, req):
        ret, _ = self.write_ptz(req.ptz)
        if not ret:
            rospy.logwarn('Failed to write PTZ cmd via service. Device busy.')
        return WritePTZResponse(ret)


    def write_ptz_action_callback(self, goal):
        # write ptz command and calculate duration
        ts = rospy.Time.now()
        ret = self.write_ptz(goal.ptz)
        dt = (rospy.Time.now()-ts).to_sec()

        if not ret:
            rospy.logwarn(f'PTZ command failed after {dt} seconds')
            self.ptz_sas.set_aborted()
            self.reset_driver()
            return

        rospy.loginfo(f'PTZ command written successfully in {dt} seconds')

        # wait unitl goal is reached or until failure
        goal_reached, state = self.wait_ptz_goal_reached(
            goal, timeout=self.ptz_action_timeout)

        # abort action request if goal wasn't reached
        if not goal_reached:
            self.ptz_sas.set_aborted()
            self.reset_driver()
            return

        # prepare action result to return
        result = self.state_to_result(state, ts)
        rospy.sleep(rospy.Duration(2.0))
        self.ptz_sas.set_succeeded(result)

        # update stored ptz state
        self.update_state(state)

        # publish ptz state
        self.ptz_pub.publish(state)


    def update_state(self, state):
        with self.drv_lock:
            self.camera_pose = {
                'pan': state.pan,
                'tilt': state.tilt,
                'ir_zoom': state.ir_zoom,
                'vis_zoom': state.vis_zoom
            }


    def state_to_result(self, state, start_time):
        result = PTZResult()
        result.result = True
        result.ptz = state
        result.elapsed_time = state.header.stamp - start_time
        return result


    def validate_zoom(self, ir_zoom, vis_zoom):
        c1 = self.drv.validate_zoom_slaving()
        c2 = ir_zoom != vis_zoom
        c3 = not (ir_zoom == 0 and abs(vis_zoom-8.2) > 1e-1)

        return c1 and c2 and c3


    def wait_ptz_goal_reached(self, goal, timeout=10):
        start_time = rospy.Time.now()
        state, timestamp = None, None
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            ts = rospy.Time.now()
            ret, timestamp = self.read_ptz()
            dt = (rospy.Time.now()-ts).to_sec()
            if ret is None:
                rospy.logwarn('Failed to read ptz state after {dt} seconds')
                rospy.sleep(rospy.Duration(1.0))
            else:
                state = PanTiltZoom(pan=ret['pan'], tilt=ret['tilt'],
                                    ir_zoom=ret['ir_zoom'],
                                    vis_zoom=ret['vis_zoom'],
                                    header=Header(stamp=timestamp))
                self.camera_pose = ret

                if self.is_goal_reached(state, goal.ptz):
                    rospy.loginfo(
                        f'PTZ state read successfully in {dt} seconds and '
                        'goal is reached')
                    break

                rospy.loginfo(
                    f'PTZ state read successfully in {dt} seconds and '
                    'goal is not reached')
        else:
            self.ptz_status = 'PTZ goal couldn\'t be reached'
            return False, state

        return self.validate_zoom(state.ir_zoom, state.vis_zoom), state


    def is_goal_reached(self, state, goal):
        e1 = abs(state.pan - goal.pan) < self.ptz_tolerance
        e2 = abs(state.tilt - goal.tilt) < self.ptz_tolerance
        e3 = abs(state.ir_zoom - goal.ir_zoom) < self.ptz_tolerance
        #  e4 = abs(state.vis_zoom - goal.vis_zoom) < self.ptz_tolerance
        return e1 and e2 and e3 # and e4


    def ptz_to_msg(self, pan, tilt, ir_zoom, vis_zoom, timestamp):
        msg = PanTiltZoom()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id
        msg.pan = pan
        msg.tilt = tilt
        msg.ir_zoom = ir_zoom
        msg.vis_zoom = vis_zoom
        return msg
