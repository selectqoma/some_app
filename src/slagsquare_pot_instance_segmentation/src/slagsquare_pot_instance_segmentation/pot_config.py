import os
import sys

# Import Mask RCNN Config
from mrcnn.config import Config

############################################################
#  Configurations
############################################################

class PotConfig(Config):
    """Configuration for training on the pot dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "pot"

    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

    BATCH_SIZE = 1

    # Number of classes (including background)
    # NUM_CLASSES = 1 + 2  # Background + pot + kaizer_pot
    NUM_CLASSES = 1 + 1  # Background + pot

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 10

    BACKBONE = "resnet50"

    # Skip detections with less than 85% confidence
    DETECTION_MIN_CONFIDENCE = 0.9

    DETECTION_NMS_THRESHOLD = 0.3

    # Input image resizing
    IMAGE_RESIZE_MODE = "square"
    IMAGE_MIN_DIM = 704
    IMAGE_MAX_DIM = 704
    #IMAGE_MIN_SCALE = 2.0
    IMAGE_PADDING = True

    # Length of square anchor side in pixels
    RPN_ANCHOR_SCALES = (8, 16, 32, 64, 128)

    # ROIs kept after non-maximum suppression
    POST_NMS_ROIS_TRAINING = 2000
    POST_NMS_ROIS_INFERENCE = 4000

    # Non-max suppression threshold to filter RPN proposals.
    # You can increase this during training to generate more propsals.
    RPN_NMS_THRESHOLD = 1.0

    # How many anchors per image to use for RPN training
    RPN_TRAIN_ANCHORS_PER_IMAGE = 512

    # If enabled, resizes instance masks to a smaller size to reduce
    # memory load. Recommended when using high-resolution images.
    USE_MINI_MASK = True
    #  MINI_MASK_SHAPE = (56, 56)  # (height, width) of the mini-mask
    #  MASK_SHAPE = [28, 28]

    # Number of ROIs per image to feed to classifier/mask heads
    # The Mask RCNN paper uses 512 but often the RPN doesn't generate
    # enough positive proposals to fill this and keep a positive:negative
    # ratio of 1:3. You can increase the number of proposals by adjusting
    # the RPN NMS threshold.
    TRAIN_ROIS_PER_IMAGE = 1024

    # Maximum number of ground truth instances to use in one image
    MAX_GT_INSTANCES = 100
    # Max number of final detections per image
    DETECTION_MAX_INSTANCES = 100

    WEIGHT_DECAY = 0.01
