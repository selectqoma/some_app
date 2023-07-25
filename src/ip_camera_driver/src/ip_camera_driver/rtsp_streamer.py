import sys
import time
import numpy as np
import contextlib
import subprocess
from datetime import datetime
from collections import namedtuple, deque
import tempfile
from pathlib import Path
import uuid
import contextlib
import os
from threading import Lock
import cv2

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from ip_camera_driver.utils import calculate_fps

StreamProperties = namedtuple('StreamProperties',
                              ['width', 'height', 'framerate'])

DEQUE_SIZE = 10


@contextlib.contextmanager
def temp_fifo():
    with tempfile.TemporaryDirectory() as tmpdir:
        fifo_name = str(uuid.uuid4()) + '.fifo'
        fifo_path = Path(tmpdir) / fifo_name
        os.mkfifo(fifo_path)
        yield fifo_path


class _VlcStreamer:

    def __init__(self, port=8554):
        self.template = (
            'cvlc --demux=rawvid '
            '--aout=alsa '
            '--rawvid-width={width:.0f} --rawvid-height={height:.0f} '
            '--rawvid-fps={framerate:.1f} --rawvid-chroma=RV24 '
            '{path_in} --sout-keep --no-sout-all --sout {output!r}'
        )

        self.outputs = {
            'display': '#display',
            'local': f'#transcode{{vcodec=h264}}:rtp{{sdp=rtsp://127.0.0.1:{port}/}}',
            'remote': f'#transcode{{vcodec=h264}}:rtp{{sdp=rtsp://:{port}/}}',
            'both': ('#duplicate{dst=display,'
                     f'dst="transcode{{vcodec=h264}}:rtp{{sdp=rtsp://:{port}/}}"}}'),
        }


    def stream(self, stream_properties, output='remote'):
        with temp_fifo() as fifo_path:
            width, height, framerate = stream_properties
            command = self.template.format(
                width=width,
                height=height,
                framerate=framerate,
                path_in=fifo_path,
                output=self.outputs[output],
            )
            rospy.loginfo(f'Launching {command!r}')

            with contextlib.suppress(BrokenPipeError), \
                    subprocess.Popen(command, shell=True), \
                    open(fifo_path, 'wb') as stream_input:
                time.sleep(0.5)
                while True:
                    image = yield
                    stream_input.write(
                        cv2.cvtColor(image, cv2.COLOR_BGR2RGB).tobytes())


class FrameStreamer:

    def __init__(self):
        self.load_params()
        self.initialize_vars()
        self.initialize_ros_interfaces()
        self.initialize_timers()


    def load_params(self):
        self.output = rospy.get_param('~output', 'display')
        self.port = rospy.get_param('~port', 554)
        self.scale = rospy.get_param('~scale', 1.0)
        self.show_timestamp = rospy.get_param('~show_timestamp', False)
        self.framerate = rospy.get_param('~framerate', 1)
        self.expected_input_fps = rospy.get_param('~expected_input_fps', 0)


    def initialize_vars(self):
        self.streamer = _VlcStreamer(port=self.port)
        self.bridge = CvBridge()
        self.image_lock = Lock()
        self.stream = None
        self.image = None
        self.counter_in = 0
        self.counter_out = 0
        self.timestamps_in = deque([], maxlen=DEQUE_SIZE)
        self.timestamps_out = deque([], maxlen=DEQUE_SIZE)


    def initialize_ros_interfaces(self):
        self.image_sub = rospy.Subscriber(
            'image_raw', Image, self.image_callback, queue_size=1)
        self.diagnostics_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=1)


    def initialize_timers(self):
        self.stream_timer = \
            rospy.Timer(rospy.Duration(1.0/self.framerate),
                        self.stream_callback)
        self.diagnostics_timer = \
            rospy.Timer(rospy.Duration(1.0), self.diagnostics_callback)


    def image_callback(self, image_msg):
        self.image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        self.timestamps_in.append(image_msg.header.stamp)
        self.counter_in += 1


    def stream_callback(self, event):
        with self.image_lock:
            if self.image is None:
                return

            image = self.image.copy()
            timestamp = self.timestamps_in[-1]

        h, w = image.shape[:2]
        image = cv2.resize(image, (int((w//2)*2*self.scale),
                                   int((h//2*2)*self.scale)),
                          interpolation=cv2.INTER_AREA)

        # overlay red mask to indicate system failure
        if (rospy.Time.now() - timestamp).to_sec() > 5 * 60:
            overlay = np.zeros_like(image)
            overlay[:, :, 2] = 255
            cv2.addWeighted(image, 0.5, overlay, 0.5, 1, image)

        if self.show_timestamp:
            time = datetime.now().strftime("%H:%M:%S")
            cv2.putText(image, time, (int(2*w/5),int(h/20)),
                       cv2.FONT_HERSHEY_SIMPLEX, w/1000, (0,0,255), 1,
                       cv2.LINE_AA)

        if self.stream is None:
            height, width = image.shape[:2]
            framerate = self.framerate
            stream_properties = StreamProperties(width, height, framerate)
            self.stream = self.streamer.stream(
                stream_properties, output=self.output)
            next(self.stream)
        try:
            self.stream.send(image)
            self.counter_out += 1
            self.timestamps_out.append(rospy.Time.now())
        except StopIteration:
            pass


    def diagnostics_callback(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        msg.status.append(self.get_input_diagnostic_status())
        msg.status.append(self.get_output_diagnostic_status())
        self.diagnostics_pub.publish(msg)


    def get_input_diagnostic_status(self):
        fps = calculate_fps(self.timestamps_in)

        if len(self.timestamps_in) > 0:
            last_update = self.timestamps_in[-1].to_sec()
        else:
            last_update = 0

        status = DiagnosticStatus()
        status.name = rospy.get_namespace()[1:] + 'rtsp_streamer/input'
        status.hardware_id = ''
        status.values.append(KeyValue(key='fps', value=str(fps)))
        status.values.append(
            KeyValue(key="last_update",
                     value=str(last_update)))
        status.values.append(
            KeyValue(key='total_frames',
                     value=str(self.counter_in)))

        if fps >= self.expected_input_fps * 0.9:
            status.level = DiagnosticStatus.OK
            status.message = 'Input framerate within range (' + \
                f'{self.expected_input_fps} fps)'
        elif 0 < fps < self.expected_input_fps * 0.9:
            status.level = DiagnosticStatus.WARN
            status.message = 'Input framerate is below threshold (' + \
                f'{self.expected_input_fps} fps)'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Input framerate is {fps} fps'

        return status


    def get_output_diagnostic_status(self):
        fps = calculate_fps(self.timestamps_out)

        if len(self.timestamps_out) > 0:
            last_update = self.timestamps_out[-1].to_sec()
        else:
            last_update = 0

        status = DiagnosticStatus()
        status.name = rospy.get_namespace()[1:] + 'rtsp_streamer/output'
        status.hardware_id = ''
        status.values.append(KeyValue(key='fps', value=str(fps)))
        status.values.append(
            KeyValue(key="last_update",
                     value=str(last_update)))
        status.values.append(
            KeyValue(key='total_frames',
                     value=str(self.counter_out)))

        if fps >= self.framerate * 0.9:
            status.level = DiagnosticStatus.OK
            status.message = 'Output framerate within range (' + \
                f'{self.framerate} fps)'
        elif 0 < fps < self.framerate * 0.9:
            status.level = DiagnosticStatus.WARN
            status.message = 'Output framerate is below threshold (' + \
                f'{self.framerate} fps)'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Output framerate is {fps} fps'

        return status
