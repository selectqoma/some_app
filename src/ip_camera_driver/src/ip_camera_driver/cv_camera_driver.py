import os
import cv2
import sys
import rospy
import rospkg
import actionlib
from collections import deque
from copy import deepcopy
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Empty
from polled_camera.srv import GetPolledImage, GetPolledImageResponse
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from smelter_actions.msg import TakeSnapshotAction, TakeSnapshotResult

from ip_camera_driver.image_conversions import raw_msg_to_compressed
from ip_camera_driver.utils import calculate_fps

if sys.version_info[0] == 2:
    from camera_calibration_parsers import readCalibration
else:
    def readCalibration(path):
        rospy.logwarn('Loading of camera info is not supported for python3')
        return None

# fix error of opencv using tcp for rtsp capture
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

encodings = {'8UC1': 'mono8', '8UC3': 'bgr8'}


class CVCameraDriver(object):

    def __init__(self):
        # initialize member variables
        self.camera_info = None
        self.cap = None
        self.seq = 0
        self.cv_bridge = CvBridge()
        self.frame_lock = Lock()
        self.latest_frame = None
        self.latest_published_frame = None
        self.connection_status = 'N/A'
        self.capture_status = 'N/A'
        self.input_timestamps = deque([], maxlen=10)
        self.output_timestamps = deque([], maxlen=10)

        # load parameters
        self.load_params()

        # create publishers
        self.diagnostics_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10)
        self.frame_pub = rospy.Publisher('image_raw', Image, queue_size=10)
        self.compressed_frame_pub = rospy.Publisher(
            'image/compressed',
            CompressedImage, queue_size=10)

        if self.manual_acquisition:
            self.snapshot_pub = rospy.Publisher('snapshot/image_raw',
                                                Image, queue_size=10)
            self.compressed_snapshot_pub = rospy.Publisher(
                'snapshot/image/compressed',
                CompressedImage, queue_size=10)

        # open connection to camera
        self.open_video_capture()
        self.frame_grabbing_thread = rospy.Timer(
            rospy.Duration(1), self.grab_frames, oneshot=True)
        self.frame_publishing_thread = rospy.Timer(
            rospy.Duration(1), self.publish_frame_thread, oneshot=True)

        # start diagnostics timer
        self.diagnostics_timer= rospy.Timer(
            rospy.Duration(1.0), self.diagnostics_callback)

        if self.manual_acquisition:
            self.rf_sub = rospy.Subscriber(
                'take_snapshot', Empty, self.read_frame_callback)
            self.rf_srv = rospy.Service(
                'take_snapshot', GetPolledImage, self.read_frame_srv_callback)
            self.rf_sas = actionlib.SimpleActionServer(
                'take_snapshot', TakeSnapshotAction,
                execute_cb=self.read_frame_action_callback,
                auto_start=False)
            self.rf_sas.start()
        else:
            # create timer to read and publish camera frames
            self.read_timer = rospy.Timer(rospy.Duration(1.0/self.framerate),
                                          self.read_frame_callback)
            # create timer to publish camera info
            if self.camera_info is not None:
                self.camera_info_pub = rospy.Publisher(
                    'camera_info', CameraInfo, queue_size=10)
                self.camera_info_timer = rospy.Timer(
                    rospy.Duration(1e-2), self.publish_camera_info_callback)
            else:
                rospy.logwarn('Failed to find an intrinsics calibration file')


    def load_params(self):
        # load params from param server
        self.camera_name = rospy.get_param('~camera_name')
        self.frame_id = rospy.get_param('~frame_id')
        self.ip = rospy.get_param('~uri/ip')
        self.protocol = rospy.get_param('~uri/protocol')
        self.user = rospy.get_param('~uri/username')
        self.password = rospy.get_param('~uri/password')
        self.uri_suffix = rospy.get_param('~uri/suffix')
        self.port = rospy.get_param('~uri/port')
        self.framerate = rospy.get_param('~settings/framerate')
        self.manual_acquisition = rospy.get_param(
            '~settings/manual_acquisition', False)
        self.mock = rospy.get_param('~mock', False)

        # load camera info from yaml file
        '''
        Disabled due to issue with cv_bridge and python. Importing
        python3-cv_bridge causes calibration parsers to not be able to find the
        camera info yaml file, so a separate script is used to publish the
        calibration files
        '''
        self.camera_info = self.read_camera_info(self.camera_name)

        # create the camera uri
        self.cam_uri = '{}://{}:{}@{}:{}/{}'.format(
            self.protocol, self.user, self.password, self.ip,
            self.port, self.uri_suffix)


    def read_camera_info(self, camera_name):
        cam_info_path = os.path.join(
            rospkg.RosPack().get_path('ip_camera_driver'),
            'calibrations', camera_name+'_intrinsic_calibration.yaml')
        camera_info = None
        try:
            ret = readCalibration(cam_info_path)
            if ret is None:
                raise ValueError('Camera Info is None')
            camera_info = ret[1]
            rospy.loginfo('Loaded camera calibration for camera {} from path '
                          '{}'.format(camera_name, cam_info_path))
        except Exception as e:
            rospy.logwarn(
                'Failed to find a camera calibration file for camera '
                '{} in path {} with error {}'.format(
                    camera_name, cam_info_path, e))
        return camera_info


    def diagnostics_callback(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()

        status = DiagnosticStatus()
        status.name = self.camera_name
        status.level = DiagnosticStatus.OK
        status.message = 'OK'

        status.values.append(KeyValue(
            key='connection_status', value=self.connection_status))
        status.values.append(KeyValue(
            key='capture_status', value=self.capture_status))

        input_fps = calculate_fps(self.input_timestamps)
        output_fps = calculate_fps(self.output_timestamps)

        if input_fps > 0.9 * self.framerate:
            input_fps_status = 'OK'
        else:
            status.level = DiagnosticStatus.ERROR
            input_fps_status = f'Below threshold {input_fps} < ' + \
                f'{self.framerate}'

        if output_fps > 0.9 * self.framerate:
            output_fps_status = 'OK'
        else:
            status.level = DiagnosticStatus.ERROR
            output_fps_status = f'Below threshold {output_fps} < ' + \
                f'{self.framerate}'

        status.values.append(KeyValue(
            key='input_fps_status', value=input_fps_status))
        status.values.append(KeyValue(
            key='output_fps_status', value=output_fps_status))

        if not input_fps_status or not output_fps_status:
            status.message = 'Failure'

        msg.status.append(status)
        self.diagnostics_pub.publish(msg)


    def grab_frames(self, event):
        while not rospy.is_shutdown():
            if not self.cap.isOpened() or not self.grab_frame():
                self.connection_status = 'Failure'
                self.open_video_capture()
                rospy.sleep(0.5)
            else:
                self.connection_status = 'OK'


    def grab_frame(self):
        frame_msg = self.read_frame()
        if frame_msg is None:
            self.capture_status = 'Failure'

            return False
        else:
            self.capture_status = 'OK'
            with self.frame_lock:
                self.latest_frame = frame_msg
                self.input_timestamps.append(frame_msg.header.stamp)

            return True


    def publish_frame_thread(self, event):
        rate = rospy.Rate(self.framerate)
        latest_published_frame = None
        while not rospy.is_shutdown():
            with self.frame_lock:
                if self.latest_frame != latest_published_frame:
                    latest_published_frame = deepcopy(self.latest_frame)
                else:
                    continue
            self.output_timestamps.append(latest_published_frame.header.stamp)
            self.publish_frame(latest_published_frame)
            rate.sleep()


    def publish_frame(self, frame):
        self.frame_pub.publish(frame)
        compressed = raw_msg_to_compressed(frame)
        self.compressed_frame_pub.publish(compressed)


    def publish_snapshot(self, snapshot):
        self.snapshot_pub.publish(snapshot)
        compressed = raw_msg_to_compressed(snapshot)
        self.compressed_snapshot_pub.publish(compressed)


    def open_video_capture(self):
        if self.cap is None:
            self.cap = VideoCaptureWrapper(
                self.cam_uri, self.camera_name, self.mock)
        else:
            self.cap.reset()

        if self.mock:
            rospy.loginfo('Opened video capture for mock camera '
                          f'{self.camera_name}')
            return True
        elif self.cap.isOpened():
            rospy.loginfo(f'Opened video capture for {self.cam_uri}')
            return True
        else:
            rospy.loginfo(f'Failed to open video capture for {self.cam_uri}')
            return False


    def get_latest_frame(self):
        if self.mock:
            self.grab_frame()

        with self.frame_lock:
            if self.latest_frame is not None and \
                    self.latest_frame != self.latest_published_frame:
                self.latest_published_frame = self.latest_frame
                return deepcopy(self.latest_frame)
            else:
                return None


    def read_frame_callback(self, event):
        frame_msg = self.get_latest_frame()
        if frame_msg is not None:
            self.frame_pub.publish(frame_msg)


    def read_frame_srv_callback(self, req):
        response = GetPolledImageResponse()
        response.stamp = self.frame_stamp
        frame_msg = self.get_latest_frame()
        if frame_msg is not None:
            self.frame_pub.publish(frame_msg)
            response.success = True
        else:
            response.success = False
        return response


    def read_frame_action_callback(self, goal):
        rospy.loginfo('Received action goal to take snapshot for camera '
                      '{}'.format(self.camera_name))
        with self.frame_lock:
            self.cap.ptz = [goal.ptz.pan, goal.ptz.tilt, goal.ptz.ir_zoom]

        rospy.sleep(0.1)

        frame_msg = self.get_latest_frame()

        if frame_msg is not None:
            self.publish_snapshot(frame_msg)
            r = TakeSnapshotResult()
            r.success = True
            r.snapshot = frame_msg
            self.rf_sas.set_succeeded(r)
        else:
            rospy.logerr('Failed to read frame from camera {}. '
                         'Aborting request.'.format(self.camera_name))
            self.rf_sas.set_aborted()


    def read_frame(self):
        timestamp = rospy.Time.now()
        ret, frame = self.cap.read()

        if not ret:
            return None

        try:
            frame_msg = self.cv_bridge.cv2_to_imgmsg(
                frame, encoding="passthrough")
        except CvBridgeError as e:
            rospy.logwarn(
                f'Failed to convert cv img to ros msg with exception {e}')
            return None

        # fill frame message
        frame_msg.header.stamp = timestamp
        frame_msg.header.seq = self.seq + 1
        self.seq += 1
        frame_msg.header.frame_id = self.frame_id
        frame_msg.encoding = encodings[frame_msg.encoding]

        return frame_msg


    # publish camera_info
    def publish_camera_info_callback(self, timer):
        self.camera_info.header.stamp = rospy.Time.now()
        self.camera_info.header.seq += 1
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info_pub.publish(self.camera_info)


class VideoCaptureWrapper:

    def __init__(self, cam_uri, camera_name, mock):
        self.mock = mock
        self.camera_name = camera_name
        self.cam_uri = cam_uri
        self.path = None
        self.frames = []
        self.ptz = None  # [0.0, 0.0, 0.0]
        if self.mock:
            subpath = 'rgb' * ('rgb' in self.camera_name) + \
                'ir' * ('rgb' not in self.camera_name)
            self.path = os.path.join(
                rospkg.RosPack().get_path('smelter_datasets_and_models'),
                'slagsquare_monitoring', 'patrol_frames', subpath)
            self.frame_filenames = os.listdir(self.path)
        else:
            self.cap = cv2.VideoCapture(self.cam_uri)


    def isOpened(self):
        if self.mock:
            return True
        else:
            return self.cap.isOpened()


    def reset(self):
        self.cap = cv2.VideoCapture(self.cam_uri)


    def read(self):
        ret, frame = False, None
        if self.mock:

            if self.ptz is not None:
                filename = None
                for fn in self.frame_filenames:
                    if "{}_{}_{}".format(*self.ptz) in fn:
                        filename = fn
                        break
                else:
                    return None, None
                path = os.path.join(self.path, filename)
                frame = cv2.imread(path, cv2.IMREAD_COLOR)
                ret = True
        else:
            ret, frame = self.cap.read()

        return ret, frame
