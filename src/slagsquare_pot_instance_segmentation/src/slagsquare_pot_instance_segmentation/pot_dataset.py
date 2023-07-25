import os
import cv2
import sys
import json
import datetime
import numpy as np
from xml.dom import minidom
import math
from mrcnn import utils
import importlib

class PotDataset(utils.Dataset):

    def load_pot(self, dataset_dir, subset):
        """Load a subset of the Pot dataset.
        dataset_dir: Root directory of the dataset.
        subset: Subset to load: train or val
        """
        # Add classes. We have only one class to add.
        self.classes = ['BG', 'pot']
        self.add_class("pot", 1, "pot")
        #self.add_class("pot", 2, "kaizer_pot")

        # Load annotations
        filename = 'pot_point_annotations.xml'
        annotations = self.load_annotations_from_xml(dataset_dir, filename)

        # Train or validation dataset?
        #  assert subset in ["train", "val"]
        dataset_dir = os.path.join(dataset_dir, subset)
        filenames = os.listdir(dataset_dir)

        # Add images
        for a in annotations:
            if a['filename'] not in filenames:
                continue
            ellipses = [pot['points'] for pot in a['pots']]
            ellipses = []
            for pot in a['pots']:
                points = np.array(pot['points'])
                polygon = points
                ellipses += [polygon]

            labels = [pot['type'] for pot in a['pots']]
            int_labels = []
            for label in labels:
                if label == 'pot':
                    int_labels += [1]
                elif label == 'kaizer_pot':
                    # int_labels += [2]
                    int_labels += [1]
                else:
                    raise ValueError('Wrong type of annotation: {}'.format(label))


            image_path = os.path.join(dataset_dir, a['filename'])
            height, width = a['height'], a['width']
            self.add_image(
                "pot",
                image_id=a['id'],#a['filename'],  # use file name as a unique image id
                path=image_path,
                width=width, height=height,
                polygons=ellipses,
                labels=int_labels)

    def load_annotations_from_xml(self, path, filename):
        xml_annotations = minidom.parse(os.path.join(path, filename)).getElementsByTagName('image')
        annotations = []
        for a in xml_annotations:
            img_id = a.attributes['id'].value
            img_name = a.attributes['name'].value
            img_width = int(a.attributes['width'].value)
            img_height = int(a.attributes['height'].value)

            pots = []
            for pot in a.childNodes[1::2]:
                pot_type = pot.childNodes[1].firstChild.data
                pot_points = []
                for pixel in  pot.attributes['points'].value.split(';'):
                    x, y = pixel.split(',')
                    pot_points += [[int(float(x)), int(float(y))]]
                if len(pot_points) < 5:
                    print('Warning found pot with less than five points: %d'%len(pot_points))
                    continue
                pot_points = np.array(pot_points)
                # convert annotated points into ellipses
                try:
                    ellipse, (xc,yc,a,b,theta) = self.fit_ellipse(pot_points, 30)
                except ValueError as e:
                    ####################### DEBUG IMG SAVE #####################
                    debug_img = np.zeros((img_height, img_width))
                    ellipse, (xc,yc,a,b,theta) = self.fit_ellipse(pot_points, 30, exception=False)
                    cv2.ellipse(debug_img, (int(xc),int(yc)), (int(a/2),int(b/2)), int(theta), 0, 360, color=(255,)*3, thickness=1)
                    for pt in pot_points:
                        cv2.circle(debug_img, tuple(pt), 5, (255,)*3, -1)
                    cv2.imwrite('/home/gkouros/Desktop/debug.png', debug_img)
                    ############################################################
                    raise ValueError('Failed to fit ellipse for pot:\n{}\nin image {} with error:\n\n{}'.format(pot.toprettyxml(), img_name, e))

                pots += [{'type': pot_type, 'points': ellipse}]

            annotations += [{'filename': img_name, 'id': img_id, 'width': img_width, 'height': img_height, 'pots': pots}]

        return annotations


    def load_mask(self, image_id):
        """Generate instance masks for an image.
        Returns:
            masks: A bool array of shape [height, width, instance count] with
                one mask per instance.
            class_ids: a 1D array of class IDs of the instance masks.
        """
        # If not a pot dataset image, delegate to parent class.
        info = self.image_info[image_id]
        if info["source"] != "pot":
            return super(self.__class__, self).load_mask(image_id)

        # Convert polygons to a bitmap mask of shape
        # [height, width, instance_count]
        info = self.image_info[image_id]
        mask = np.zeros([info["height"], info["width"], len(info["polygons"])],
                        dtype=np.uint8)

        for i, polygon in enumerate(info['polygons']):
            mask[:,:,i] = cv2.fillConvexPoly(mask[:,:,i], polygon, (1,)*3).get()

        # map class names to class IDs
        class_ids = info['labels']

        # Return mask, and array of class IDs of each instance. Since we have
        # one class ID only, we return an array of 1s
        return mask.astype(np.bool), np.array(class_ids).astype(np.int32)

    def image_reference(self, image_id):
        """Return the path of the image."""
        info = self.image_info[image_id]
        if info["source"] == "pot":
            return info["path"]
        else:
            super(self.__class__, self).image_reference(image_id)

    def rotate_points(self, x, y, xo, yo, theta):
        xr = int(math.cos(theta)*(x-xo)-math.sin(theta)*(y-yo) + xo)
        yr = int(math.sin(theta)*(x-xo)+math.cos(theta)*(y-yo) + yo)
        return [xr,yr]

    def fit_ellipse(self, points, num_interp_pts=10, exception=True):
        ellipse = []
        (xc, yc), (a, b), theta = cv2.fitEllipseDirect(points)
        if exception:
            if max(a/2,b/2) > 200:
                raise ValueError("Too big ellipse: a={}, b={}".format(a,b))
        t = np.linspace(0, 2*math.pi, num_interp_pts)
        X = xc + a / 2 * np.cos(t)
        Y = yc + b / 2 * np.sin(t)
        for pt in zip(X,Y):
            x, y = pt
            xr, yr = self.rotate_points(x, y, xc, yc, theta*math.pi/180)

            ellipse += [[int(round(xr)),int(round(yr))]]

        return np.array(ellipse), (xc, yc, a, b, theta)
