import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

def cv2_img_to_compressed(img):
    ret, compressed = cv2.imencode('.jpg', img)
    return ret, compressed

def cv2_compressed_to_msg(img, timestamp=None, frame_id='camera'):
    msg = CompressedImage()
    if timestamp is None:
        msg.header.stamp = rospy.Time.now()
    else:
        msg.header.stamp = timestamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = np.array(img).tostring()
    return msg

def compressed_msg_to_cv2(img_msg, channels=1):
    np_arr = np.fromstring(img_msg.data, np.uint8)
    image_np = None
    if channels == 1:
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
    elif channels == 3:
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    else:
        raise ValueError('Requested to decode image with {channels} '
                         'channels. Expected 1 or 3.')
    return image_np

def compressed_msg_to_raw(img, channels=1):
    raw_np = compressed_msg_to_cv2(img, channels=channels)
    raw = CvBridge().cv2_to_imgmsg(img, encoding="passthrough")
    return raw

def raw_msg_to_compressed(img_msg):
    img = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
    ret, compressed = cv2_img_to_compressed(img)
    if not ret:
        rospy.logerr('Failed to encode image')
        return None
    compressed_msg = cv2_compressed_to_msg(compressed, img_msg.header.stamp,
                                           img_msg.header.frame_id)
    return compressed_msg


