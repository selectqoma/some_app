import os
import cv2
import numpy as np
import rospy
import rospkg
import rosparam
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest
from sensor_msgs.msg import Image

class FumesCoverMonitor:

    __font = cv2.FONT_HERSHEY_SIMPLEX
    __origin = (100,20)
    __fontScale = 0.5
    __color = (255, 255, 255)
    __thickness = 1

    def __init__(self, mock=True):
        self.read_params()
        self.last_cover_state_on = None
        if mock:
            return
        self.cv_bridge = CvBridge()
        try:
            rospy.wait_for_service('~start', timeout=self.wait_service_timeout)
            rospy.wait_for_service('~stop', timeout=self.wait_service_timeout)
            self.start_srv = rospy.ServiceProxy('~start', Empty)
            self.stop_srv = rospy.ServiceProxy('~stop', Empty)
        except rospy.ServiceException as e:
            reason = f'Failed to create ros service clients for starting ' \
                'stopping tapping monitor with error: {e}'
            rospy.signal_shutdown(reason)

        # subscriber must be initialized before services to avoid receiving a message before service client starts
        self.img_sub = rospy.Subscriber('~image_raw', Image, self.image_callback, queue_size=10)
        self.img_pub = rospy.Publisher('~fumes_cover_state/image_raw', Image, queue_size=10)
        self.fumes_cover_state_pub = rospy.Publisher('~fumes_cover_on', Bool, queue_size=10)

    def read_params(self):
        pkg_path = rospkg.RosPack().get_path('cascade_state_monitoring')
        filepath = os.path.join(pkg_path, 'config', 'fumes_cover_monitor_params.yaml')
        try:
            params = rosparam.load_file(filepath)[0][0]
            self.intensity_threshold = int(params['intensity_threshold'])
            self.wait_service_timeout = int(params['wait_service_timeout'])
            self.pixel_percentage_threshold = int(params['pixel_percentage_threshold'])
            self.fumes_cover_on_roi = params['fumes_cover_on_roi']
            self.fumes_cover_off_rois = params['fumes_cover_off_rois']
        except rosparam.RosParamException as e:
            error = f'Cannot load param with error {e}'
            rospy.logerr(error)
            rospy.signal_shutdown(error)

    def image_callback(self, img_msg):
        img = self.cv_bridge.imgmsg_to_cv2(img_msg)
        self.read_params()
        self.process_img(img)
        annot_img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")

        try:
            self.img_pub.publish(annot_img_msg)
        except rospy.exceptions.ROSException as e:
            rospy.signal_shutdown(f'Failed to publish with error: {e}')

    def process_img(self, img):
        cover_state_on = self.is_fumes_cover_on(img)
        if self.last_cover_state_on == None and cover_state_on == True:
            self.start_srv(EmptyRequest())
        if self.last_cover_state_on == False and cover_state_on == True:
            self.start_srv(EmptyRequest())
        elif self.last_cover_state_on == True and cover_state_on == False:
            self.stop_srv(EmptyRequest())
        self.last_cover_state_on = cover_state_on
        self.fumes_cover_state_pub.publish(cover_state_on)

    def is_fumes_cover_on(self, img):
        on = self. process_roi(img, self.fumes_cover_on_roi)
        off = any([self.process_roi(img, roi) for roi in self.fumes_cover_off_rois])

        ret = False
        if on and not off:
            cv2.putText(img,'Fumes Cover is ON',self.__origin, self.__font,
                        self.__fontScale,self.__color,self.__thickness,cv2.LINE_AA)
            ret = True
        elif not on and off:
            cv2.putText(img,'Fumes Cover is OFF',self.__origin, self.__font,
                        self.__fontScale,self.__color,self.__thickness,cv2.LINE_AA)
            ret = False
        else:
            cv2.putText(img,'Fumes Cover State Unknown. Presumed OFF.',
                        self.__origin,self.__font,self.__fontScale,self.__color,
                        self.__thickness,cv2.LINE_AA)
            ret = False

        return ret

    def process_roi(self, img, roi):
        x,y,w,h = roi['x'], roi['y'], roi['width'], roi['height']
        roi_img = img[y:y+h, x:x+w]
        _,th_img = cv2.threshold(roi_img, self.intensity_threshold, 255, cv2.THRESH_BINARY)
        percentage = np.count_nonzero(th_img)/np.prod(th_img.shape)
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,255),1)
        return percentage > self.pixel_percentage_threshold
