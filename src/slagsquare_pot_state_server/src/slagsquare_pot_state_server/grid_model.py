import numpy as np
import cv2
from functools import partial
from collections import deque

from slagsquare_pot_state_server.occupancy_map import OccupancyMap
from slagsquare_pot_state_server.grid_model_utils import debugLog, loadYaml, \
    getContourCenters, getProjectedCenters, getCentersImageColored, \
    getValidationImage, getExplicitValidationImage, getTemperatureData, \
    annotateIrImg


class GridModel(object):
    def __init__(
            self,
            h_ir2rgb_path,
            h_rgb2cad_path,
            rows=8,
            cols=20,
            debug=False):

        self.debug = debug
        self.centerColor = (255, 0, 0)
        self.contourColor = (0, 255, 0)
        self.centerThickness = 7

        self.homographiesIrRgb = {}
        self.homographiesRgbCad = {}

        self.occupancyMap = None # influenced by (1 or 2) CraneObservers
        self.potTemperatures = None

        self.centersImg = None
        self.heatMaps = deque([], maxlen=100)

        self.rows = rows
        self.cols = cols

        self.rgbImageWidth = 0
        self.rgbImageHeight = 0
        self.cadImage = None
        self.cadImageWidth = 0
        self.cadImageHeight = 0

        self.debugImagesCenters = deque([], maxlen=100)
        self.debugImagesIrCadWarps = deque([], maxlen=100)
        self.latest_pot_slots = None

        # validation function parameters
        self.radius = 0
        self.validationMargin = 0.85
        self.validationImage = None

        self.loadHomographiesFromYaml(h_ir2rgb_path, h_rgb2cad_path)

        # column offset parameters
        self.patrolMaskCenters = deque([], maxlen=len(self.homographiesIrRgb))
        self.useColumnOffsets = False
        self.columnOffsets = -.125 * np.ones(self.cols) # start with an upward bias

        # kaiser parameters
        self.kaiserRow = 7
        self.kaiserRightColumnOffset = 5


    def loadHomographiesFromYaml(self, pathIrRgbYaml, pathRgbCadYaml):
        self.homographiesIrRgb = loadYaml(pathIrRgbYaml)
        self.homographiesRgbCad = loadYaml(pathRgbCadYaml)
        debugLog(self.debug, 'Homography dictionaries loaded.')


    def setPartialFunctions(self):
        '''
        Sets the grid and validation functions based on
        current parameters.
        '''
        self.gridFunction = partial(self.getGridCo,
                                    grid_model_parameters=self.cadGridMappingParameters,
                                    useColumnOffsets=self.useColumnOffsets)
        self.validationFunction = partial(self.validateMeasurement,
                                          validationRadius=self.radius*self.validationMargin,
                                          useColumnOffsets=self.useColumnOffsets)
        # self.validationImage = getValidationImage(self.cadImage, self.validationFunction)
        self.validationImage = getExplicitValidationImage(self.cadImageHeight,
                                                          self.cadImageWidth,
                                                          self.rows,
                                                          self.cols,
                                                          self.radius*self.validationMargin,
                                                          self.useColumnOffsets,
                                                          True,
                                                          self.kaiserRightColumnOffset,
                                                          self.columnOffsets)


    def setCadParameters(self, pathCadImage):
        '''Sets parameters of the CAD image for correct transformations.'''
        self.cadImage = cv2.imread(pathCadImage)
        self.cadImageHeight = self.cadImage.shape[0]
        self.cadImageWidth = self.cadImage.shape[1]
        self.rowHeight = self.cadImageHeight/float(self.rows)
        self.colWidth = self.cadImageWidth/float(self.cols)
        self.radius = ((self.cadImageWidth/self.cols/2.0) +
                       (self.cadImageHeight/self.rows/2.0))/2.0
        self.setPartialFunctions()

        debugLog(self.debug, 'CAD image loaded.')
        debugLog(self.debug,  '--- set CAD image (H,W): '
                             f'{self.cadImageHeight,self.cadImageWidth}.')
        debugLog(self.debug, f'--- rowHeight: {self.rowHeight}, '
                             f'colWidth: {self.colWidth}')
        debugLog(self.debug, f'--- radius (mean): {self.radius}, '
                             f'with validation margin: {self.validationMargin}')


    def setRgbParameters(self, rgbImageHeight: int, rgbImageWidth: int):
        self.rgbImageHeight = rgbImageHeight
        self.rgbImageWidth = rgbImageWidth
        debugLog(self.debug, f'Set RGB image (h,w):{self.rgbImageHeight,self.rgbImageWidth}.')


    def setOccupancyGrid(self):
        '''Sets parameters of the occupancy grid model.'''
        self.occupancyMap = OccupancyMap(self.rows,
                                         self.cols,
                                         self.cadImageWidth,
                                         self.cadImageHeight,
                                         self.rgbImageWidth,
                                         self.rgbImageHeight)
        self.potTemperatures = np.zeros((self.rows, self.cols))
        debugLog(self.debug, 'Created occupancy grid.')


    def validateMeasurement(self,
                            v: float,
                            u: float,
                            validationRadius: float,
                            useColumnOffsets: bool=False) -> bool:
        '''
        Returns whether a point maps to a valid grid coordinate.
        Uses a radial criterium with a variable margin.
        Note: used as a partial function.
        '''
        # y,x = self.getGridCo(v,u,useColumnOffsets)
        y,x = self.gridFunction(y=v,x=u)
        if y == None or x == None:
            return False
        assert (y >= 0 and y < self.rows)
        assert (x >= 0 and x < self.cols)
        cellCenterY = self.rowHeight//2 + self.rowHeight * y

        # kaisers
        if self.isKaiser(y,x):
            kaiserWidth = self.colWidth*1.5
            cellCenterX = kaiserWidth//2 + kaiserWidth * x
            distancePointToCellCenter = np.sqrt(((cellCenterX-u)**2)/2.+(cellCenterY-v)**2)
        else:
            cellCenterX = self.colWidth//2 + self.colWidth * x
            if useColumnOffsets:
                cellCenterY += int(self.columnOffsets[x]*self.rowHeight)
            distancePointToCellCenter = np.sqrt((cellCenterX-u)**2+(cellCenterY-v)**2)

        return distancePointToCellCenter <= validationRadius


    def isKaiser(self, y: int, x: int) -> bool:
        '''Given grid coordinates calculated normally, are we dealing with kaiser pots?'''
        return y == self.kaiserRow and x < self.cols-self.kaiserRightColumnOffset


    def calculateColumnOffsets(self):
        '''
        Calculate column offsets according to a collection of points.
        maskCenters is a (n,2)-shape np.array. Each coordinate is expected
        to be of the form [u,v].
        '''

        try:
            maskCenters = np.vstack(self.patrolMaskCenters)
        except ValueError:
            # If we get a ValueError, there might have been one or more poses that did not contain any validated mask centers.
            # It does not make sense to calculate columns offsets due to this missing information. Therefore, return reset offsets.
            return np.zeros(self.cols)

        if maskCenters.shape[1] > 0:
            # filter out kaiser pots
            maskCenters = maskCenters[[not self.isKaiser(*self.gridFunction(y=row[1],x=row[0])) for row in maskCenters]]
            i = 0
            intercepts = []
            boundaries_x = np.linspace(0,self.cadImageWidth,self.cols+1)

            maxSteps = 50
            lr = 0.01

            for i in range(self.cols):
                # get all maskcenters that fall in a column
                # maskcenters near the boundaries are NOT ignored
                y = maskCenters[(maskCenters[:,0] > boundaries_x[i]) & ((maskCenters[:,0] < boundaries_x[i+1]))]

                y[:,0] = (y[:,1]/self.rowHeight-self.columnOffsets[i]).astype(int)

                intercept = self.rowHeight/2.0

                for _ in range(maxSteps):
                    y_hat = self.rowHeight*y[:,0]+intercept # Ax+B
                    delta = lr * np.sum((y_hat-y[:,1]))
                    if abs(delta) > 1e-4:
                        intercept -= delta
                    else:
                        break
                intercepts.append(intercept)

            return (np.asarray(intercepts) - self.rowHeight/2.0) / self.rowHeight
        else:
            return self.columnOffsets


    def updateState(self, pose, rgbContours, ir):
        self.pan, self.tilt, self.zoom = pose

        # get rgb centers of contours
        centersRgb = getContourCenters(rgbContours)

        # RGB to CAD
        centersCad, hRgbCad = self.rgb2cad(pose, centersRgb)
        self.patrolMaskCenters += [centersCad]

        # create cad image with projected centers
        self.centersImg = getCentersImageColored(
                self.validationImage,
                centersCad,
                self.validationFunction)

        # RGB to IR
        #  centersIr, hIrRgb = self.rgb2ir(pose, centersRgb)
        hIrRgb = self.homographiesIrRgb[pose]

        # IR to CAD
        warpedIr = self.ir2cad(ir, hIrRgb, hRgbCad)
        self.heatMaps.append(warpedIr)

        # caclualate ir contours
        irContours = ()
        for contour in rgbContours:
            irContours += (self.rgb2ir(pose, contour)[0],)

        # update pot temperatures for currently viewed pots
        self.updatePotTemperatures(ir, irContours, warpedIr, centersCad)

        # annotate ir image
        irContoursImg = annotateIrImg(
            ir, irContours, self.latest_pot_slots)

        # update occupancy grid
        validationCount = self.occupancyMap.updateOccMap(hIrRgb, 
                                                         hRgbCad,
                                                         centersCad,
                                                         self.gridFunction,
                                                         self.validationFunction)

        return validationCount, irContoursImg


    def updatePotTemperatures(self, ir, irContours, warpedIr, centersCad):
        pot_slots = [None] * len(centersCad)
        for idx, (c, contour) in enumerate(zip(centersCad, irContours)):
            if self.validationFunction(c[1], c[0]):
                row, col = self.gridFunction(y=c[1],x=c[0])
                if col >= 20 or row >= 8: #  (raphael:) this should be removed
                    print('Pot out of grid error')
                    continue
                #  self.potTemperatures[row, col] = blurHeatMap[c[1], c[0]]
                min_, max_, mean = getTemperatureData(ir, contour)
                self.potTemperatures[row, col] = max_
                pot_slots[idx] = [row, col]
        self.latest_pot_slots = pot_slots


    def rgb2ir(self, pose, centersRgb):
        hIrRgb = self.homographiesIrRgb[pose]
        debugLog(self.debug, f'Homography IR to RGB:\n {hIrRgb}.')
        hRgbIR = np.linalg.inv(hIrRgb)
        centersIr = getProjectedCenters(centersRgb, hRgbIR)
        return centersIr, hIrRgb


    def rgb2cad(self, pose, centersRgb):
        hRgbCad = self.homographiesRgbCad[pose]
        debugLog(self.debug, f'Homography RGB to CAD:\n {hRgbCad}.')
        centersCad = getProjectedCenters(centersRgb, hRgbCad)
        return centersCad, hRgbCad


    def ir2cad(self, ir, hIrRgb, hRgbCad):
        im_warped = cv2.warpPerspective(
            ir, hIrRgb, (self.rgbImageWidth, self.rgbImageHeight))
        im_warped = cv2.warpPerspective(
            im_warped, hRgbCad, (self.cadImageWidth, self.cadImageHeight))
        return im_warped


    def checkPoseInHomographies(self, pose):
        return pose in self.homographiesIrRgb \
            and pose in self.homographiesRgbCad


    def setOccupancy(self,row,col,state):
        '''Updates the occupancy map with a strong belief.'''
        if state == 0:
            self.occupancyMap.updateNegDeduction(row,col)
        elif state == 1:
            self.occupancyMap.updatePosDeduction(row,col)
        else:
            raise ValueError("Occupancy state needs to be either 0 or 1.")


    def setCraneGridMappingParameters(self, gridModelParameters: dict):
        self.crane_x_start = gridModelParameters['x_start']
        self.crane_x_end = gridModelParameters['x_end']
        self.crane_y_start = gridModelParameters['y_start']
        self.crane_y_end = gridModelParameters['y_end']

    @property
    def craneGridMappingParameters(self):
        """Returns relevant parameters for calculating grid logic."""
        return {
            'x_start':  self.crane_x_start,
            'x_end':    self.crane_x_end,
            'y_start':  self.crane_y_start,
            'y_end':    self.crane_y_end,
        }

    @property
    def cadGridMappingParameters(self):
        """Returns relevant parameters for calculating grid logic."""
        return {
            'x_start':  0,
            'x_end':    self.cadImageWidth,
            'y_start':  0,
            'y_end':    self.cadImageHeight,
        }


    def getGridCo(self,
                  grid_model_parameters: dict,
                  y: float, x: float,
                  enableKaisers=True,
                  useColumnOffsets=None):
        """Returns (column,row) coordinates for a specific grid.

        If the coordinates are out of bounds, returns [None,None].

        Args:
            grid_model_parameters (dict): Parameters that define the grid.
            y (float):                    y in crane coordinates.
            x (float):                    x in crane coordinates.
            enable_kaisers (bool):        Take kaiser pots into account.
            useColumnOffsets (bool):      Use column offsets.

        Returns:
            y,x [int,int]:  List of x and y in grid coordinates.
        """

        if useColumnOffsets is None:
            useColumnOffsets = self.useColumnOffsets

        row_height = (grid_model_parameters['y_end']-grid_model_parameters['y_start']) \
                     /self.rows
        col_width = (grid_model_parameters['x_end']-grid_model_parameters['x_start']) \
                    /self.cols

        y -= grid_model_parameters['y_start']
        x -= grid_model_parameters['x_start']

        y_binned = int(y/row_height)
        x_binned = int(x/col_width)

        if enableKaisers and self.isKaiser(y_binned,x_binned):
            kaiserWidth = col_width*1.5
            x_binned = int(x/kaiserWidth) # recalc x coordinate
        elif useColumnOffsets:
            if x_binned >= 0 and x_binned < self.cols:
                y_binned = int((y/row_height)-self.columnOffsets[x_binned])

        if y_binned < 0 or y_binned >= self.rows or \
           x_binned < 0 or x_binned >= self.cols:
            return [None,None]

        return [y_binned,x_binned]