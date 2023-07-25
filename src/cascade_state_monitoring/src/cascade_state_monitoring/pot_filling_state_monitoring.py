import os
import cv2
import numpy as np
import rospy
import rospkg
import rosparam
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
from subprocess import check_output


class PotFillingStateMonitor():

    __scanline_colormap = {'safe': (255,255,255),
                           'warning': (0,255,255),
                           'danger': (0,0,255)}

    def __init__(self):
        self.pkg_path = rospkg.RosPack().get_path('cascade_state_monitoring')
        try:
            self.git_version = check_output(['git',
                                             f'--git-dir={self.pkg_path}/../../.git',
                                             'describe',
                                             '--broken',
                                             '--dirty',
                                             '--tags']).decode('ascii').strip()
        except:
            self.git_version = "-- Tag not found --"
            rospy.logwarn('No tag found in this commit. A new version should '
                          'not be deployed without a tag.')

        try:
            self.git_hash = check_output([
                'git',
                f'--git-dir={self.pkg_path}/../../.git',
                'rev-parse', '--verify',
                'HEAD']).decode('ascii').strip()
        except:
            self.git_hash = "-- Hash not found --"

        self.cv_bridge = CvBridge()
        self.img_sub = None
        self.annot_img_pub = None
        self.tapping_start_time = None
        self.tapping_latest_time = None
        self.states = None
        self.spouts = None
        self.scanlines = None
        self.scanlines_flags = None
        self.monitoring_state = None
        self.read_params()
        self.reset_states()
        self.start_monitoring_srv = rospy.Service('~start_monitoring', Empty, self.start_monitoring)
        self.stop_monitoring_srv = rospy.Service('~stop_monitoring', Empty, self.stop_monitoring)
        self.img_sub = rospy.Subscriber('~image_raw', Image, self.image_cb, queue_size=1)
        self.annot_img_pub = rospy.Publisher('~pot_states/image_raw', Image, queue_size=1)


    def read_params(self):
        self.read_config_params()
        self.read_spouts()


    def read_spouts(self):
        filepath = os.path.join(self.pkg_path, 'config', 'pot_spout_positions.yaml')
        try:
            spout_params = rosparam.load_file(filepath)[0][0]['spouts']
        except rosparam.RosParamException as e:
            error = f'Cannot find pot spouts configuration file with error {e}'
            rospy.logerr(error)
            rospy.signal_shutdown(error)
        self.spouts = np.array(spout_params)


    def read_config_params(self):
        filepath = os.path.join(self.pkg_path, 'config', 'pot_filling_state_monitoring_params.yaml')
        try:
            params = rosparam.load_file(filepath)[0][0]
            self.warn_flow_deviation_threshold = params['flow_warning_threshold']
            self.danger_flow_deviation_threshold = params['flow_danger_threshold']
            self.melt_pixel_threshold = params['melt_pixel_threshold']
        except rosparam.RosParamException as e:
            error = f'Cannot find configuration file with error {e}'
            rospy.logerr(error)
            rospy.signal_shutdown(error)


    def start_monitoring(self, req):
        self.tapping_start_time = rospy.Time.now()
        self.reset_states()
        self.monitoring_state = True
        return EmptyResponse()


    def reset_states(self):
        self.states = [['unknown' for _ in range(3)] for _ in range(9)]
        self.scanlines = [0] * 3
        self.scanlines_flags = ['safe'] * 3
        self.monitoring_state = False


    def stop_monitoring(self, req):
        self.monitoring_state = False
        return EmptyResponse()


    def image_cb(self, data):
        # update spouts
        try:
            self.read_params()
        except rosparam.RosParamException as e:
            # when editing a file and saving it, it's temporarily unavailable
            rospy.logwarn('Cannot find pot spouts configuration file')
            pass
        # convert to cv2 using cv_bridge
        img = self.cv_bridge.imgmsg_to_cv2(data)
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # monitor states
        if self.monitoring_state:
            annot_img = self.update_states(img)
        else:
            annot_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            self.draw_annotations(annot_img)

        # convert back to Image msg and publish annotated image
        annot_img_msg = self.cv_bridge.cv2_to_imgmsg(annot_img)
        annot_img_msg.encoding = 'bgr8'
        annot_img_msg.header.stamp = data.header.stamp
        self.annot_img_pub.publish(annot_img_msg)


    def update_states(self, img):
        # update the states
        h,w = 3, 20
        max_progress = max(self.scanlines)
        for col in range(len(self.scanlines)):
            while self.scanlines[col] < 8:
                spout_col = self.spouts[col]
                x = spout_col[self.scanlines[col]+1][0]
                y = spout_col[self.scanlines[col]+1][1]
                roi = img[y-h:y+h, x-w:x+w]
                if roi.max() > self.melt_pixel_threshold:
                    self.states[self.scanlines[col]+1][col] = 'filling'
                    self.states[self.scanlines[col]][col] = 'filled'
                    self.scanlines[col] += 1
                else:
                    break

            if self.scanlines[col] > max_progress - self.warn_flow_deviation_threshold:
                self.scanlines_flags[col] = 'safe'
            elif self.scanlines[col] > max_progress - self.danger_flow_deviation_threshold:
                self.scanlines_flags[col] = 'warning'
            else:
                self.scanlines_flags[col] = 'danger'

        # draw annotations on the input image
        annot_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        self.draw_annotations(annot_img)
        return annot_img


    def draw_annotations(self, img):
        self.draw_spout_graph(img)
        self.draw_scanlines(img)
        self.draw_states_panel(img)
        self.draw_legend(img)
        self.draw_monitoring_state_annotation(img, (100, 380))
        self.draw_time(img, (300, 380))
        self.draw_warning(img, (80, 440))
        self.draw_version(img, (20, 470))


    def draw_spout_graph(self, img):
        for spout_col in self.spouts:
            cv2.polylines(img, [spout_col], False, (255,0,0), 1)
            for spout in spout_col:
                cv2.circle(img, tuple(spout), 2, (0,255,0), -1)


    def draw_scanlines(self, img):
        for col in range(len(self.scanlines)):
            pt1 = self.spouts[col, self.scanlines[col]] + np.array([-10, 0])
            pt2 = self.spouts[col, self.scanlines[col]] + np.array([10, 0])
            cv2.line(img, tuple(pt1), tuple(pt2),
                     self.__scanline_colormap[self.scanlines_flags[col]], 2)


    def draw_states_panel(self, img):
        # draw description text
        cv2.putText(img, 'Filling States', (50, 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255,255,255), 1, lineType=cv2.LINE_AA)

        # draw panel border
        top_left_corner = np.array([70,40])
        h_offset = 20
        v_offset = 20
        cv2.rectangle(img, tuple(top_left_corner-np.array([v_offset, h_offset])),
                      tuple(top_left_corner + np.array([len(self.states[0])*v_offset, len(self.states)*h_offset])),
                      (255,255,255), 0, cv2.LINE_AA)

        # draw pot state indicators
        circle_size = 8
        circle_outline_color = (127,)*3
        for i in range(len(self.states)):
            for j in range(len(self.states[0])):
                position = top_left_corner + np.array([j*v_offset, i*h_offset])
                if self.states[i][j] == 'filled':
                    cv2.circle(img, tuple(position), circle_size, (255,)*3, -1, cv2.LINE_AA)
                    cv2.circle(img, tuple(position), circle_size, circle_outline_color, 1, cv2.LINE_AA)
                elif self.states[i][j] == 'filling':
                    cv2.ellipse(img, tuple(position), (circle_size, circle_size), 0, 0, 180, (255,)*3, -1, cv2.LINE_AA)
                    cv2.ellipse(img, tuple(position), (circle_size, circle_size), 0, 0, -180, (127,)*3, 1, cv2.LINE_AA)
                elif self.states[i][j] == 'unknown':
                    cv2.circle(img, tuple(position), circle_size, (127,)*3, -1, cv2.LINE_AA)
                    cv2.circle(img, tuple(position), circle_size, circle_outline_color, 1, cv2.LINE_AA)
                else:
                    print('Unrecognized state: %s'%states[i][j])
        self.draw_legend(img)


    def draw_legend(self, img):
        # add legend text
        position = (500,20)
        h_offset = 20
        v_offset = 20
        circle_size = 8
        circle_outline_color = (127,)*3
        cv2.putText(img, 'Legend', tuple(position+np.array([25,-5])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, lineType=cv2.LINE_AA)
        cv2.rectangle(img, tuple(position), tuple(position + np.array([5*v_offset, 7*h_offset])), (255,255,255), 0, cv2.LINE_AA)
        # filled pot - white circle
        symbol_pos = position+np.array([h_offset, v_offset])
        text_pos = (position+np.array([round(1.8*h_offset), round(1.2*v_offset)])).astype(np.int)
        cv2.circle(img, tuple(symbol_pos), circle_size, (255,)*3, -1, cv2.LINE_AA)
        cv2.circle(img, tuple(symbol_pos), circle_size, circle_outline_color, 1, cv2.LINE_AA)
        cv2.putText(img, 'Filled Pot', tuple(text_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, lineType=cv2.LINE_AA)
        # filling pot - half filled circle
        symbol_pos = position+np.array([h_offset, 2*v_offset])
        text_pos = (position+np.array([round(1.8*h_offset), round(2.2*v_offset)])).astype(np.int)
        cv2.ellipse(img, tuple(symbol_pos), (circle_size, circle_size), 0, 0, 180, (255,)*3, -1, cv2.LINE_AA)
        cv2.ellipse(img, tuple(symbol_pos), (circle_size, circle_size), 0, 0, -180, (127,)*3, 1, cv2.LINE_AA)
        cv2.putText(img, 'Filling Pot', tuple(text_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, lineType=cv2.LINE_AA)
        # unknown/unfilled pot - empty circle
        symbol_pos = position+np.array([h_offset, 3*v_offset])
        text_pos = (position+np.array([round(1.8*h_offset), round(3.2*v_offset)])).astype(np.int)
        cv2.circle(img, tuple(symbol_pos), circle_size, (127,)*3, -1, cv2.LINE_AA)
        cv2.circle(img, tuple(symbol_pos), circle_size, circle_outline_color, 1, cv2.LINE_AA)
        cv2.putText(img, 'Empty Pot', tuple(text_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, lineType=cv2.LINE_AA)
        # stage of filling - white line
        symbol_pos = (position+np.array([h_offset//2, round(4.05*v_offset)])).astype(np.int)
        text_pos = (position+np.array([round(1.8*h_offset), round(4.2*v_offset)])).astype(np.int)
        w, h = 20, 3
        cv2.line(img, tuple(symbol_pos), tuple(symbol_pos+np.array([w, 0])), (255,)*3, h)
        cv2.putText(img, 'Safe', tuple(text_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, lineType=cv2.LINE_AA)
        # spill warning - yellow line
        symbol_pos = (position+np.array([h_offset//2, round(5.05*v_offset)])).astype(np.int)
        text_pos = (position+np.array([round(1.8*h_offset), round(5.2*v_offset)])).astype(np.int)
        w, h = 20, 3
        cv2.line(img, tuple(symbol_pos), tuple(symbol_pos+np.array([w, 0])), (0,255,255), h)
        cv2.putText(img, 'Warning', tuple(text_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, lineType=cv2.LINE_AA)
        # spilling error - red line
        symbol_pos = (position+np.array([h_offset//2, round(6.05*v_offset)])).astype(np.int)
        text_pos = (position+np.array([round(1.8*h_offset), round(6.2*v_offset)])).astype(np.int)
        w, h = 20, 3
        cv2.line(img, tuple(symbol_pos), tuple(symbol_pos+np.array([w, 0])), (0,0,255), h)
        cv2.putText(img, 'Danger', tuple(text_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1, lineType=cv2.LINE_AA)


    def draw_time(self, img, pos):
        if self.monitoring_state:
            self.tapping_latest_time = rospy.Time.now()
        if self.tapping_start_time is None or self.tapping_latest_time is None:
            duration = 0
        else:
            duration = (self.tapping_latest_time - self.tapping_start_time).to_sec()

        hh = int(duration // 3600)
        mm = int(duration // 60)
        ss = int(duration % 60)
        cv2.putText(img, 'Tapping Elapsed Time: %02d:%02d:%02d'%(hh, mm, ss),
                    pos, cv2.FONT_HERSHEY_DUPLEX, 0.5, (200,)*3, 1,
                    cv2.LINE_AA)


    def draw_monitoring_state_annotation(self, img, pos):
        if self.monitoring_state:
            status = 'Tapping'
        else:
            status = 'Inactive'
        cv2.putText(img, f'Status: {status}', pos, cv2.FONT_HERSHEY_DUPLEX, 0.5,
                    (200.0,)*3, 1, lineType=cv2.LINE_AA)


    def draw_warning(self, img, pos):
        x0, y0 = pos
        img[y0-20:y0+10, x0-10:x0+500, :] = [0, 255, 255]
        cv2.putText(img, 'Automatic video analytics does not replace manual '
                    'control !', pos, cv2.FONT_HERSHEY_DUPLEX, 0.5,
                    (0.0,0.0,0.0), 1, lineType=cv2.LINE_AA)


    def draw_version(self, img, pos):
        x0, y0 = pos
        cv2.putText(img, 'Version: Alpha '+self.git_version, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255.0,255.0,255.0), 1, lineType=cv2.LINE_AA)
        cv2.putText(img, self.git_hash, (x0+300, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255.0,255.0,255.0), 1, lineType=cv2.LINE_AA)
