#!/usr/bin/env python3

import os
import sys

import mrcnn
from mrcnn.config import Config
from mrcnn import model as modellib, utils

from slagsquare_pot_instance_segmentation.pot_dataset import PotDataset
from slagsquare_pot_instance_segmentation.pot_config import PotConfig

import importlib

# get location of MRCNN lib
ROOT_DIR = os.environ['MASK_RCNN_ROOT_DIR']
DATASET_DIR = os.environ['POT_RGB_INSTANCE_SEGMENTATION_DATASET_ROOT_DIR']

config = PotConfig()
config.display()
model_dir = ""
logs_dir = os.path.join(ROOT_DIR, "logs")
weights_path = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")

model = modellib.MaskRCNN(mode="training", config=config, model_dir=logs_dir)
model.load_weights(weights_path, by_name=True, exclude=[
    "mrcnn_class_logits", "mrcnn_bbox_fc",
    "mrcnn_bbox", "mrcnn_mask"])

dataset_train = PotDataset()
dataset_train.load_pot(DATASET_DIR, "train")
dataset_train.prepare()

dataset_val = PotDataset()
dataset_val.load_pot(DATASET_DIR, "val")
dataset_val.prepare()

#  model.train(dataset_train, dataset_val, learning_rate=config.LEARNING_RATE,
        #  epochs=10, layers="heads")
