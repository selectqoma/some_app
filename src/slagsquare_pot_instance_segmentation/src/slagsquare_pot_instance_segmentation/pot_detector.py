""" pot_detector.py: Contains a class that implements a pot detector using
Mask-RCNN for instance segmentation
"""

import os
import sys
import cv2
import numpy as np

import rospy
import rospkg
import actionlib
#  import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from smelter_msgs.msg import (Contour, ContourArray, Rect, RectArray,
                              Point2D, DetectedObjects, ClassIds)
from std_msgs.msg import Empty
from smelter_actions.msg import DetectObjectsAction, DetectObjectsResult, \
    DetectObjectsGoal
from slagsquare_pot_instance_segmentation.pot_config import PotConfig

#  from mrcnn.config import Config
from mrcnn import model as modellib
from mrcnn.visualize import random_colors
#  from mrcnn.model import log, utils
#  from mrcnn import visualize

from tensorflow.keras.backend import clear_session
import tensorflow.compat.v1 as tf
#  tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
#  tf.disable_v2_behavior()

clear_session()
tf.reset_default_graph()

global SESSION
SESSION = tf.Session(graph=tf.Graph())

sys.dont_write_bytecode = True

class InferenceConfig(PotConfig):
    """ The configuration class for inference mode
    """
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    RPN_NMS_THRESHOLD = 1.0


class PotDetector:
    """ The pot detector class that uses Mask-RCNN for instance segmentation
    """

    def __init__(self) -> None:
        """ Initializes pot detector
        """
        self.load_params()
        self.cv_bridge = CvBridge()
        self.init_pub = rospy.Publisher('pot_detector/initialized', Empty, queue_size=1)
        self.initialize_detector()
        self.cam_info = CameraInfo()

        # initialize subscribers
        if self.auto_mode:
            self.img_sub = rospy.Subscriber(
                'image_raw', Image, self.image_callback, queue_size=1)
            #  self.img_sub = message_filters.Subscriber(
                #  '~rgb/image', Image, queue_size=1)
            #  self.cam_info_sub = message_filters.Subscriber(
                #  '~rgb/camera_info', CameraInfo, queue_size=1)
            #  self.sync = message_filters.ApproximateTimeSynchronizer(
                #  [self.img_sub, self.cam_info_sub], 100, 5.0)
            #  self.sync.registerCallback(self.callback)
            #  self.cam_info_sub = rospy.Subscriber(
            #      '~rgb/camera_info', CameraInfo,
            #      self.camera_info_callback, queue_size=1)
        else:
            self.detect_pots_sas = actionlib.SimpleActionServer(
                'detect_pots', DetectObjectsAction,
                execute_cb=self.detect_pots_action_callback, auto_start=False)
            self.detect_pots_sas.start()

        # initialize publishers
        self.obj_pub = rospy.Publisher(
            'detected_pots', DetectedObjects, queue_size=10)
        self.annot_img_pub = rospy.Publisher(
            'detected_pots/image_raw', Image, queue_size=1)


    def load_params(self) -> None:
        """ Loads node parameters from the param server
        """
        self.auto_mode = self.load_param('~auto_mode')
        model_pkg = self.load_param('~model_pkg')
        model_path = self.load_param('~model_path')
        pkg_path = rospkg.RosPack().get_path(model_pkg)
        self.model_path = os.path.join(pkg_path, model_path)


    def load_param(self, param: str):
        """ Wrapper of get_param that provides better debug info

        Args:
            param (str): The name of the parameter to load

        Returns:
            param_value: The value of the parameter
        """
        if rospy.has_param(param):
            param_value = rospy.get_param(param)
        else:
            rospy.logerr(f'No {param} parameter was specified. '
                         'Unable to initialize node.')
            sys.exit()

        return param_value


    def initialize_detector(self) -> None:
        """ Initializes the Mask-RCNN detector
        """
        DEVICE = "/device:CPU:0"
        global SESSION

        with tf.device(DEVICE):
            with SESSION.as_default():
                with SESSION.graph.as_default():
                    self.inference_config = InferenceConfig()
                    self.detector = modellib.MaskRCNN(
                        mode="inference",
                        config=self.inference_config,
                        model_dir=self.model_path)
                    self.detector.load_weights(self.model_path, by_name=True)

                    # first detection for initialization
                    _ = self.detect_pots(np.zeros((704, 480, 3)))

        rospy.logwarn('Pot Detector Initialized')

        # publish empty message that signals that the node has been initialized
        self.init_pub.publish(Empty())


    #  def camera_info_callback(self, cam_info: CameraInfo) -> None:
        #  """ Receives a camera info message only once and then deactivates
        #  """
        #  self.cam_info = cam_info
        #  self.img_sub = rospy.Subscriber(
            #  'image_raw', Image, self.image_callback, queue_size=1)
        #  self.cam_info_sub.unregister()


    def callback(self, img_msg: Image, cam_info: CameraInfo) -> None:
        """ Listens for synchronized Image and CameraInfo messages to
        process and publish the detections

        Args:
            img_msg (Image): The image message to detect pots in
            cam_info (CameraInfo): The intrinsics of the camera that captured
            the image
        """
        self.process_image(img_msg=img_msg, cam_info=cam_info)


    def image_callback(self, img_msg: Image) -> None:
        """ Listens for Image messages to process and publish the detections

        Args:
            img_msg (Image): The image message to detect pots in
            cam_info (CameraInfo): The intrinsics of the camera that captured
            the image
        """
        self.process_image(img_msg=img_msg, cam_info=CameraInfo())


    def detect_pots_action_callback(self, goal: DetectObjectsGoal) -> None:
        """ Listens for and serves action goal requests to detect pots

        Args:
            goal (DetectObjectsGoal): The action goal to serve that includes
            an image to perform pot detection on
        """
        r = DetectObjectsResult(success=False)

        try:
            r.detected_objects = self.process_image(goal.image, self.cam_info)
            r.success = True
        except Exception as e:
            rospy.logerr('Failed to detect pots with exception {}'.format(e))

        if r.success:
            self.detect_pots_sas.set_succeeded(r)
        else:
            self.detect_pots_sas.set_aborted()


    def process_image(self, img_msg, cam_info):
        """ Detects and publishes pots

        Args:
            img_msg (Image): The image message to detect pots in
            cam_info (CameraInfo): The intrinsics of the camera that captured
            the image
        """
        # convert image to cv2 using cv bridge
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, "passthrough")

        # feed image to instance segmentation network to get masks
        ts = rospy.Time.now()
        masks, rois, class_ids, scores = self.detect_pots(img)
        d = (rospy.Time.now() - ts).to_sec()

        rospy.loginfo('Pot Detection took {} seconds for frame #{}'.format(
            d, img_msg.header.seq))

        # return if no detections
        if len(scores) == 0:
            rospy.logwarn('No detections')

        # convert masks to contours msg
        contours_msg, contours = self.convert_masks_to_contours_msg(masks)

        # convert rois to msg
        rois_msg = self.convert_rois_to_msg(rois)

        # convert class_ids to message
        class_ids_msg = self.convert_class_ids_to_msg(class_ids)

        # create image and contours message
        pots_msg = DetectedObjects()
        pots_msg.header = img_msg.header
        pots_msg.image = img_msg
        pots_msg.cam_info = cam_info
        pots_msg.contours = contours_msg
        pots_msg.rois = rois_msg
        pots_msg.class_ids = class_ids_msg
        pots_msg.scores = scores

        # publish messages
        self.obj_pub.publish(pots_msg)

        # annotate and publish image
        annot_img = self.annotate_img(
            img, rois, contours, class_ids,
            ['BG', 'pot', 'kaizer_pot'], scores)
        annot_img_msg = self.cv_bridge.cv2_to_imgmsg(annot_img)
        annot_img_msg.encoding = 'bgr8'
        self.annot_img_pub.publish(annot_img_msg)

        return pots_msg


    def detect_pots(self, img):
        """ Detects pot instances using Mask-RCNN

        Args:
            img (np.array): The image to detect pots in
        """
        DEVICE = "/device:CPU:0"
        global SESSION

        # call nn detector
        with tf.device(DEVICE):
            with SESSION.as_default():
                with SESSION.graph.as_default():
                    results = self.detector.detect([img], verbose=0)[0]

        return results['masks'], results['rois'], results['class_ids'], \
            results['scores']


    def convert_masks_to_contours_msg(self, masks: np.array) \
            -> (ContourArray, np.array):
        """ Calculates contours from pot instance masks

        Args:
            masks (np.array): WxHxN where N is number of detected instances

        Returns:
            ContourArray: The detected pot contours in a ROS message
            np.array: The contours in a list
        """
        contours_msg = ContourArray()
        contours = []

        for i in range(masks.shape[2]):
            contour_msg = Contour()

            if cv2.__version__[0] == '4': # OpenCV4
                contour, _ = cv2.findContours(
                    masks[:, :, i].astype(np.uint8) * 255,
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_NONE)
            else: # OpenCV3
                _, contour, _ = cv2.findContours(
                    masks[:, :, i].astype(np.uint8) * 255,
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_NONE)

            if len(contour) > 1:
                """
                Sometimes findContours returns multiple contours for a single
                mask.  So we either keep the biggest one that is the last or
                combine all the points in a single contour
                """

                # contour = [contour[-1]]
                contour = np.vstack(contour).reshape(1, -1, 1, 2)

            # get convex hull instead of potentially non-convex shapes
            contour = cv2.convexHull(contour[0]).reshape(1, -1, 1, 2)

            # copy points into contour message
            for point in contour[0]:
                point_msg = Point2D()
                point_msg.x = point[0, 0]
                point_msg.y = point[0, 1]
                contour_msg.points.append(point_msg)

            contours_msg.contours.append(contour_msg)
            contours += [contour]

        return contours_msg, contours

    def convert_rois_to_msg(self, rois) -> RectArray:
        """ Converts ROIS from lists to ROS msg

        Args:
            rois (np.array): The ROIs (Nx4) to convert to a ROS msg

        Returns:
            RectArray: Message containing the input ROIs
        """
        msg = RectArray()

        for i in range(rois.shape[0]):
            rect = Rect()
            rect.x = rois[i, 0]
            rect.y = rois[i, 1]
            rect.width = rois[i, 2]
            rect.height = rois[i, 3]
            msg.rects.append(rect)

        return msg

    def convert_class_ids_to_msg(self, class_ids: np.array) -> ClassIds:
        """ Converts class ids from list to ROS msg

        Args:
            class_ids (np.array): The class ids to convert to a ROS message

        Returns:
            ClassIds: The class ids ROS msg
        """
        # input: Nx1 class indices
        msg = ClassIds()
        msg.class_ids = class_ids.astype(np.int8)
        return msg

    def annotate_img(self,
                     img: np.array,
                     boxes: np.array,
                     contours: np.array,
                     class_ids: np.array,
                     class_names: list,
                     scores: np.array
                     ) -> np.array:
        """ Annotates the input image with the given pot detections

        Args:
            img (np.array): The input image to annotate
            boxes (np.array): The bounding boxes of the pot detections in
                [y1,x1,y2,x2] format
            contours (np.array): The contours to draw on the image
            class_ids (np.array): The class_ids to use for drawing the correct
                name annotation for each pot
            class_names (list): The list of possible class names
            scores (np.array): The scores of each detection

        Returns:
            np.array: The annotated image
        """
        annot_img = img.copy()
        N = boxes.shape[0]
        colors = random_colors(N)

        for i in range(N):
            color = tuple([int(c*255) for c in colors[i]])
            y1, x1, y2, x2 = boxes[i]
            cv2.rectangle(annot_img, (x1, y1), (x2, y2), color, thickness=1)

            try:
                cv2.drawContours(annot_img, contours[i], -1, color, 2)
            except Exception as exc:
                pass

            cv2.putText(
                annot_img,
                class_names[class_ids[i]] + \
                str(i) + ': {:.2f}'.format(scores[i]),
                (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
                lineType=cv2.LINE_AA)

        return annot_img
