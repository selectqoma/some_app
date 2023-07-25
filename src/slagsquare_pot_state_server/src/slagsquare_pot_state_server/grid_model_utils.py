import numpy as np
import yaml
import cv2
#  import matplotlib.pyplot as plt
from numba import jit, prange
from mrcnn.visualize import random_colors


# general/helper functions
# ------------------------------------------------------

def debugLog(debug: bool, message: str):
    '''If global debug is true, print a debug message.'''
    if debug:
        print(message)


#  def debugShow(debug: bool, image: np.array):
    #  '''If global debug is true, show a debug image.'''
    #  if debug:
        #  plt.figure(figsize=(15,10))
        #  plt.imshow(image)
        #  plt.show()


def loadYaml(yamlPath):
    '''load dictionaries with matrices as values'''
    with open(yamlPath) as f:
        loaded = yaml.load(f, Loader=yaml.FullLoader)
    float_tuple_key_loaded = {}
    for k in loaded:
        fk = tuple(map(float, k))
        float_tuple_key_loaded[(round(fk[0], 2), round(
            fk[1], 2), fk[2])] = np.array(loaded[k])
        float_tuple_key_loaded[(round(fk[0] + 0.005, 2),
                                round(fk[1] + 0.005),
                                fk[2])] = np.array(loaded[k])
        float_tuple_key_loaded[(round(fk[0] - 0.005, 2),
                                round(fk[1] + 0.005),
                                fk[2])] = np.array(loaded[k])
        float_tuple_key_loaded[(round(fk[0] + 0.005, 2),
                                round(fk[1] - 0.005),
                                fk[2])] = np.array(loaded[k])
        float_tuple_key_loaded[(round(fk[0] - 0.005, 2),
                                round(fk[1] - 0.005),
                                fk[2])] = np.array(loaded[k])
    return float_tuple_key_loaded

# Calculation functions
# ------------------------------------------------------

def getMaskCenters(masks):
    contours = ()
    for i in range(masks.shape[2]):
        contour, _ = cv2.findContours(
            masks[:, :, i].astype(np.uint8) * 255, 1, 2)[0]
        contours += (contour,)
    return getContourCenters(contours)


def getContourCenters(contours):
    centers = []
    for contour in contours:
        M = cv2.moments(contour)
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append([cX, cY])
        except BaseException:
            # divide by zero error
            pass
    return np.asarray(centers)


def getProjectedCenters(centers: np.array, homography: np.array) -> np.array:
    '''
    Given a (2,n) array of center coordinates, calculate the projected centers
    according to a homography matrix.
    Note: the projected centers could lie beyond the initial borders.
    '''
    if centers is not None and centers.shape[0] > 0:
        # add homography point
        original_centers = np.hstack([centers,np.ones((centers.shape[0],1))])
        projections = np.apply_along_axis(lambda x: homography@x, 1, original_centers)
        projections = projections/projections[:,-1:] # normalize
        projections = projections.astype(int)

        return projections[:,:2]
    else:
        return np.array([])

# imshow functions
# ------------------------------------------------------

def getCentersImage(image, centers):
    annot_img = image.copy()
    for c in centers:
        cv2.circle(annot_img, (c[0],c[1]), 7, color=(0, 0, 255), thickness=7)
    return annot_img


def getCentersImageColored(image, centers, valFunc):
    annot_img = image.copy()
    for c in centers:
        if valFunc(c[1], c[0]):  # center coordinates are (X,Y)
            cv2.circle(annot_img, tuple(c), 3, color=(0, 255, 0), thickness=7)
        else:
            cv2.circle(annot_img, tuple(c), 3, color=(255, 0, 0), thickness=7)
    return annot_img


#  def showContours(image, masks):
    #  annot_img = image.copy()
    #  for i in range(masks.shape[2]):
        #  contour, _ = cv2.findContours(masks[:, :, i].astype(np.uint8) * 255, 1, 6)
        #  cv2.drawContours(annot_img, contour, 0, (255,0,0), 2)
    #  plt.figure(figsize=(10,10))
    #  plt.imshow(annot_img)
    #  plt.show()


def getValidationImage(image, validationFunction):
    '''Expensive function to visualize validation areas.'''
    validationImage = image.copy()
    for y in prange(validationImage.shape[0]):
        for x in prange(validationImage.shape[1]):
            if validationFunction(y,x):
                validationImage[y,x,:] = [0,0,255]
            else:
                validationImage[y,x,:] = [0,0,0]

    return validationImage


@jit(target='cpu',nopython=True, parallel=True)
def getExplicitValidationImage(height,
        width,
        rows,
        cols,
        validationRadius,
        useColumnOffsets,
        enableKaisers,
        kaiserRightColumnOffset,
        columnOffsets):

    '''Fast but fragile function to generate the validation map'''

    colWidth = width/float(cols)
    rowHeight = height/float(rows)
    kaiserWidthFactor = 1.5

    validationMap = np.zeros((height,width,3), np.uint8)

    for v in prange(validationMap.shape[0]):
        for u in prange(validationMap.shape[1]):
            x = int(u/colWidth)
            if useColumnOffsets:
                y = int(v/rowHeight-columnOffsets[x])
                y = max(0,y)
                y = min(y,rows-1)
            else:
                y = int(v/rowHeight)

            # kaisers, we do this at the end because we need to original y,x coordinates first
            if enableKaisers:
                if y == rows-1 and x < cols-kaiserRightColumnOffset:
                    kaiserWidth = colWidth*kaiserWidthFactor
                    x = int(u/kaiserWidth) # recalc x coordinate
                    y = int(v/rowHeight) # disregard columnoffset

            cellCenterY = rowHeight//2 + rowHeight * y

            # kaisers
            if y == rows-1 and x < cols-kaiserRightColumnOffset:
                kaiserWidth = colWidth*kaiserWidthFactor
                cellCenterX = kaiserWidth//2 + kaiserWidth * x
                distancePointToCellCenter = np.sqrt(((cellCenterX-u)**2)/2.+(cellCenterY-v)**2)
            else:
                cellCenterX = colWidth//2 + colWidth * x
                if useColumnOffsets:
                    cellCenterY += int(columnOffsets[x] * rowHeight)
                distancePointToCellCenter = np.sqrt((cellCenterX-u)**2+(cellCenterY-v)**2)

            if distancePointToCellCenter <= validationRadius:
                validationMap[v,u] = [0,0,255]

    return validationMap


def getTemperatureData(img, contour):
    mask = np.zeros(img.shape, np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    mean = cv2.mean(img, mask=mask)
    min_, max_, _, _ = cv2.minMaxLoc(img, mask=mask)
    return int(min_), int(max_), int(mean[0])


def annotateIrImg(img, contours, slots):
    """ Annotates an image with contours, temperatures and labels

        Keyword arguments:
        img -- The ir image to annotate
        contours -- The contours to draw on the image
        slos -- The labels to draw for each contour
    """
    annot_img = (255.0 * (img.copy() - img.min()) /
                 (img.max() - img.min())).astype(np.uint8)
    if len(img.shape) == 2:
        annot_img = cv2.cvtColor(annot_img, cv2.COLOR_GRAY2BGR)
    colors = np.array(random_colors(len(contours))) * 255
    for i, contour in enumerate(contours):
        x, y, w, h = cv2.boundingRect(contour)
        cv2.drawContours(annot_img, [contour], -1, colors[i], 1)
        cv2.rectangle(annot_img, (x, y), (x + w, y + h),
                      colors[i], thickness=1)
    annot_img = cv2.pyrUp(annot_img)
    for i, contour in enumerate(contours):
        if slots[i] is None:
            slot = 'N/A'
        else:
            slot = 'ABCDEFGH'[slots[i][0]] + str(slots[i][1])
        x, y, w, h = cv2.boundingRect(contour)
        cv2.putText(
            annot_img, 'pot: {}'.format(slot), (2 * x + 5 , 2 * y + 15),
            #  annot_img, 'pot: {}'.format(slot), (2 * x, 2 * y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.3, colors[i], 1,
            lineType=cv2.LINE_AA)
        min_, max_, mean = getTemperatureData(img, contour)
        cv2.putText(
            annot_img, 'T: {}'.format(max_), (2 * x + 5, 2 * y + 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.3, colors[i], 1,
            lineType=cv2.LINE_AA)

    return annot_img
