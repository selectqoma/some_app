from math import exp
import cv2
import numpy as np
from collections import deque


def logit2prob(logit) -> float:
    """ Converts logit to probability.

        Args:
            logit (float)

        Returns:
            probability (float)
    """

    return 1 / (1 + exp(-logit))


class OccupancyMap(object):

    def __init__(self,
                 rows,
                 cols,
                 refImageWidth,
                 refImageHeight,
                 maskImageWidth,
                 maskImageHeight):

        self.DEBUG = False
        self.rows = rows
        self.cols = cols
        self.refImageHeight = refImageHeight
        self.refImageWidth = refImageWidth
        self.maskImageHeight = maskImageHeight
        self.maskImageWidth = maskImageWidth
        self.map = np.zeros((rows, cols))
        # self.crane_map = np.zeros((rows, cols)) # you can use this one to debug/develop
        self.mapHistory = deque([], maxlen=100)

        self.maxLogitMrcnn = 1.7
        self.minLogitMrcnn = -1.7
        self.measurementDelta = 0.5
        self.deductionDelta = 5.

        # Occupancy parameters.
        # ---------------------------------------------------------------------
        # p(z=0|m=1) = FP/(TP+TN)
        pFreePos = 0.1
        # p(z=1|m=1) = TP/(TP+TN)
        pOccPos = 0.6
        # p(z=0|m=0) = TN/(FP+FN)
        pFreeNeg = 0.2
        # p(z=1|m=0) = FN/(FP+FN)
        pOccNeg = 0.1
        # case 1, the cells with z=1 (measurements says that it is occupied)
        self.logOddOcc = np.log(pOccPos / pOccNeg)
        # case 2, the cells with z=0 (measurement says that it is free)
        self.logOddFree = np.log(pFreeNeg / pFreePos)
        # ---------------------------------------------------------------------


    def updateOccMap(self, hIrRgb, hRgbCad, maskCentersCAD, gridFunc, valFunc) -> float:
        """Update the occupancy map beliefs based on camera measurements.

        This function maps the detections from the maskRCNN to the discrete grid and
        updates the occupancy maps accordingly. It calculates a map which in the ideal 
        world should be visible (presenceMask). Then is checks which cells have been
        actually found (seen) and calculates which cells it missed (unseen). The seen
        pots will get a positive measurements, while the unseen pots will get a negative
        measurement.

        Args:
            hIrRgb (np.array):          Homography matrix of IR to RGB of shape (3,3)
            hRgbCad (np.array):         Homography matrix of RGB to CAD of shape (3,3)
            maskCentersCad (np.array):  Coordinates of found pot centers of shape (n_pots,2)
            gridFunc (function):        Function to calculate CAD to grid mapping.
            valFunc (function):         Function to validate CAD coordinates.

        Returns:
            validation_count (float):   The fraction of pots that were seen vs seen+unseen.
        """
        # image-level initializations
        refImageShape = (self.refImageWidth, self.refImageHeight)
        visMask = np.ones((self.maskImageHeight, self.maskImageWidth))

        # grid-level initializations
        presenceMask = np.zeros((self.rows, self.cols))
        seen = np.zeros((self.rows, self.cols))

        # create look-up of mask centers
        searchableMaskCenters = [list(x) for x in list(maskCentersCAD)]

        # warp a mask of ones to see which part should be visible (in theory)
        visMaskWarped = cv2.warpPerspective(visMask, hRgbCad, refImageShape)

        # discretize the mask onto the grid and check which pots are seen
        for i in range(self.refImageWidth):
            for j in range(self.refImageHeight):
                if visMaskWarped[j, i] == 1:  # (H,W)
                    if valFunc(j, i):  # (H,W)
                        y, x = gridFunc(y=j, x=i)
                        y = min(y, self.rows - 1)
                        x = min(x, self.cols - 1)
                        presenceMask[y, x] = 1
                        if [i, j] in searchableMaskCenters:
                            seen[y, x] = 1

        validation_count = seen.sum()
        unseen = presenceMask - seen

        # smooth the priors (previous map values)
        # note: atm it only smooths if the values are also visible
        self.smoothingFactor = 0.8
        np.putmask(
            self.map,
            presenceMask == 1,
            self.map * self.smoothingFactor)
        self.updatePosMeasurement(seen)
        self.updateNegMeasurement(unseen)
        self.mapHistory.append(self.map.copy())

        return validation_count


    @property
    def probabilityMap(self):
        vlogit2prob = np.vectorize(logit2prob)
        p_ogm = vlogit2prob(self.map)
        return p_ogm


    def updatePosMeasurement(self, seen) -> None:
        """Update the pot occupancy belief with a measurement from the mrcnn.
        
        The update will be stronger if the current belief (logit) is far from
        self.maxLogitMrcnn and will be weaker if the current belief (logit) is
        close to self.maxLogitMrcnn.

        Args:
            seen (np.array): boolean map of size (rows,cols) to indicate seen cells.
        """
        weight = np.clip(self.maxLogitMrcnn - self.map,0.,1.)
        self.map += weight * self.measurementDelta * seen


    def updateNegMeasurement(self, unseen) -> None:
        """Update the pot occupancy belief with a measurement from the mrcnn.
        
        The update will be stronger if the current belief (logit) is far from
        self.minLogitMrcnn and will be weaker if the current belief (logit) is
        close to self.minLogitMrcnn.

        Args:
            unseen (np.array): boolean map of size (rows,cols) to indicate unseen cells.
        """
        weight = np.clip(self.map - self.minLogitMrcnn,0.,1.)
        self.map -= weight * self.measurementDelta * unseen


    def updatePosDeduction(self, row, col) -> None:
        """
        Update the pot occupancy belief with a deduction from the crane observer
        system. This represents a stronger belief than a mrcnn measurement.

        Args:
            row (int): Row index.
            col (int): Column index.
        """
        self.map[row,col] = self.deductionDelta


    def updateNegDeduction(self, row, col) -> None:
        """
        Update the pot occupancy belief with a deduction from the crane observer
        system.

        Args:
            row (int): Row index.
            col (int): Column index.
        """
        self.map[row,col] = -self.deductionDelta

    def print_maps(self) -> None:
        def get_char(v):
            if v > 0:
                return '▓'
            elif v == 0:
                return 'Ø'
            else:
                return '░'

        print('               maskrcnn_map              vs                 osi_map')
        print('----'*(self.cols+1))
        for y in range(self.rows):
            for x in range(self.cols):
                print(f"|{get_char(self.map[y,x])}",end='')
            print('    ',end='')
            for x in range(self.cols):
                try:
                    print(f"|{get_char(self.crane_map[y,x])}",end='')
                except:
                    pass
            print('|')
            print('----'*(self.cols+1))