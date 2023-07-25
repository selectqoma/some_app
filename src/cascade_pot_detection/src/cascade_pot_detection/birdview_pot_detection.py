import rospy
import cv2
from sensor_msgs.msg import Image
from utils.cv_bridge_conversions import CvBridgeConverter
from cascade_pot_detection.pot_detection import PotDetector
import sys
import numpy as np
sys.dont_write_bytecode = True

class BirdviewPotDetector(PotDetector):
    def __init__(self):
        PotDetector.__init__(self)
        self.detection_method_params = None
        self.detector = None # initialized only for blob detector
        self.load_params()

    def load_params(self):
        self.detection_method = rospy.get_param('~detection_method', 'contour')
        if self.detection_method is 'blob':
            detection_method_params_dict = rospy.get_param('~blob_detection_params', None)
            self.load_blob_detection_params(detection_method_params_dict)
            self.detector = cv2.SimpleBlobDetector_create(self.detection_method_params)
        elif self.detection_method is 'contour':
            detection_method_params_dict = rospy.get_param('~contour_detection_params', None)
            self.load_contour_detection_params(detection_method_params_dict)
            pass
        else:
            rospy.logwarn('Invalid detection method selected. Should be \'blob\' or \'contour\'. Using blob by default.')
            exit(0)

    def load_blob_detection_params(self, params):
        self.detection_method_params = cv2.SimpleBlobDetector_Params()
        if params is None:
            self.detection_method_params.minThreshold = 100
            self.detection_method_params.maxThreshold = 255
            self.detection_method_params.filterByArea = True
            self.detection_method_params.minArea = 1500
            self.detection_method_params.filterByCircularity = True
            self.detection_method_params.minCircularity = 0.2
            self.detection_method_params.filterByColor = True
            self.detection_method_params.blobColor = 255
            self.detection_method_params.filterByConvexity = True
            self.detection_method_params.minConvexity = 0.75
            self.detection_method_params.filterByInertia = True
            self.detection_method_params.minInertiaRatio = 0.1
        else:
            self.detection_method_params.minThreshold = params['min_threshold']
            self.detection_method_params.maxThreshold = params['max_threshold']
            self.detection_method_params.filterByArea = params['filter_by_area']
            self.detection_method_params.minArea = params['min_area']
            self.detection_method_params.filterByCircularity = params['filter_by_circularity']
            self.detection_method_params.minCircularity = params['min_circularity']
            self.detection_method_params.filterByColor = params['filter_by_color']
            self.detection_method_params.blobColor = params['blob_color']
            self.detection_method_params.filterByConvexity = params['filter_by_convexity']
            self.detection_method_params.minConvexity = params['min_convexity']
            self.detection_method_params.filterByInertia = params['filter_by_inertia']
            self.detection_method_params.minInertiaRatio = params['min_inertia_ratio']

    def load_contour_detection_params(self, params):
        self.detection_method_params = {}
        if params is None:
            self.detection_method_params['min_threshold'] = 230
            self.detection_method_params['max_threshold'] = 255
            self.detection_method_params['blur_kernel_size'] = 5
            self.detection_method_params['dilation_kernel_size'] = 3
            self.detection_method_params['dilation_iters'] = 1
            self.detection_method_params['morph_open_kernel_size'] = 11
            self.detection_method_params['roi_filters'] = [[0, 0, 40, 30], [600, 0, 40, 480]]
        else:
            self.detection_method_params['min_threshold'] = params['min_threshold']
            self.detection_method_params['max_threshold'] = params['max_threshold']
            self.detection_method_params['blur_kernel_size'] = params['blur_kernel_size']
            self.detection_method_params['dilation_kernel_size'] = params['dilation_kernel_size']
            self.detection_method_params['dilation_iters'] = params['dilation_iters']
            self.detection_method_params['morph_open_kernel_size'] = params['morph_open_kernel_size']
            self.detection_method_params['roi_filters'] = params['roi_filters']

    def detect_pots(self, img):
        if self.detection_method is 'blob':
            pots, annot_img = self.detect_pot_blobs(img)
        else:
            pots, annot_img = self.detect_pot_contours(img)
        return pots, annot_img

    def detect_pot_blobs(self, img):
        # convert to grayscale if it's not already so
        proc_img = get_gray(img)
        # clear annotations from the image
        self.filter_rois(proc_img, self.detection_method_params['roi_filters'])
        # preprocess image
        self.preprocess_img(proc_img)
        # detect blobs
        keypoints = self.detector.detect(proc_img)
        # annotate image
        annot_img = cv2.drawKeypoints(proc_img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return None, annot_img

    def detect_pot_contours(self, img):
        # convert to grayscale if it's not already so
        proc_img = self.get_gray(img)
        # clear annotations from the image
        proc_img = self.filter_rois(proc_img, self.detection_method_params['roi_filters'])
        # preprocess image
        proc_img = self.preprocess_img(proc_img)
        # find contours and draw them on the image
        _,contours,_ = cv2.findContours(proc_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # draw pot contours on annotated image
        annot_img = self.get_colored(img)
        # annotate image
        cv2.drawContours(annot_img, contours, -1, (0,255,0), 2)
        return None, annot_img

    def get_gray(self, img):
        if len(img.shape) > 2:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            return img.copy()

    def get_colored(self, img):
        if len(img.shape) < 3:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            return img.copy()

    def filter_rois(self, img, rois):
        for roi in rois:
            x,y,w,h = roi
            img[y:y+h, x:x+w] *= 0
        return img

    def preprocess_img(self, img):
        img = cv2.GaussianBlur(img, (self.detection_method_params['blur_kernel_size'],)*2, 0)
        _, img = cv2.threshold(img, self.detection_method_params['min_threshold'],
                self.detection_method_params['max_threshold'], cv2.THRESH_BINARY)
        img = cv2.dilate(img, np.ones((self.detection_method_params['dilation_kernel_size'],)*2), iterations = 1)
        kernel = np.ones((self.detection_method_params['morph_open_kernel_size'],)*2)
        img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
        return img
