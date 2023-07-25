from .crane_var import CraneVar
from .crane_states import *
from .crane_reader import CraneReader
from .grid_model import GridModel
import numpy as np

class CraneObserver:
    """Class to interpret crane movements.

    Each actual crane should have at most one crane observer,
    while each crane observer can have multiple gridmodels.

    Args:
    - name (string):            reference name of the crane
    - gridmodels (list):        list of GridModels (only works for one gridmodel atm)
    - parameter_mapping (dict): dictionary of mapping
    """
    def __init__(self, name: str, gridmodels: list, parameter_mapping: dict,
                 debug: bool = False) -> None:
        self.name = name

        self.timestamp = ''

        # position
        self.crane_x = CraneVar()
        self.crane_x_discrete = CraneVar()
        self.crane_y = CraneVar()
        self.crane_z = CraneVar()
        self.velocity_x_threshold = 1.      # min. velocity that signifies a stop
        self.velocity_y_threshold = .15

        # load weight
        self.crane_weight = CraneVar(ema_exponent = 10.)
        self.weight_threshold_empty_pot = 2500.       # load threshold that signifies a filled pot
        self.weight_threshold_filled_pot = 5000.       # load threshold that signifies a filled pot

        # states
        self.sp = StateSpace()
        self.crane_state = self.sp.STOPPED
        self.load_state = self.sp.NONE
        self.pot_state = self.sp.UNKNOWN

        # special zones
        self.zone = 0

        # pot selection
        self.pot_searching_distance = 1.5

        # hot pot dropoffs
        self.hot_pot_dropoff = {}
        self.hot_pot_cascade_pickup_info = (None,None,None)
        self.hot_pot_reposition_window = False
        self.hot_pot_last_dropoff = (None,None,None,None)

        # emptying info
        self.pot_slagsquare_pickup_info = (None,None,None)
        self.empty_pot_dropoff = {}
        self.empty_pot_last_dropoff = (None,None,None,None)

        # other
        self.debug = debug

        #
        self.parameter_mapping = parameter_mapping

        # develop
        # TODO: right now it only takes in the first gridmodel
        self.gm = gridmodels[0]


    def read(self, source):
        """Reads osi pi tags and updates crane parameters.

        Args:
        - source (dict): dictionary of osi pi values
        """
        osi_timestamp, osi_points = source

        self.timestamp = osi_timestamp

        self.crane_x.update(osi_points[self.parameter_mapping['x']])
        self.crane_x_discrete.update(osi_points[self.parameter_mapping['x_discrete']])
        self.crane_y.update(osi_points[self.parameter_mapping['y']])
        self.crane_z.update(osi_points[self.parameter_mapping['z']])
        self.crane_weight.update(osi_points[self.parameter_mapping['w']])


    def process(self, source):
        """Processes information and updates the internal state.

        This function holds the majority of the logical if-then rules
        in order to interpret and track crane movements and actions.
        The crane logic can identify the following actions:

        *. (ss means slagsquare)
        1. hot pots coming from the cascade and being placed on the ss
        2. hot pots being repositioned after being placed on the ss
        3. pots picked up from the ss, emptied and then placed back on the ss
        4. empty pot pickup and placement on the ss

        Args:
        - source (dict): dictionary of osi pi values
        """
        self.read(source)
        self.update_zone()

        # update crane movement
        if abs(self.crane_x.value_d_smooth) > self.velocity_x_threshold \
        or abs(self.crane_y.value_d_smooth) > self.velocity_y_threshold:
            self.update_crane_state_to(self.sp.MOVING)
        else:
            self.update_crane_state_to(self.sp.STOPPED)

        # update the hot pot repositioning window
        if self.hot_pot_reposition_window:
            if abs(self.hot_pot_last_dropoff[1] - self.crane_x.value) > 2.:
                self.hot_pot_reposition_window = False
            if abs(self.hot_pot_last_dropoff[3] - self.crane_y.value) > 2.:
                self.hot_pot_reposition_window = False

        # a load state should only change when the crane is stopped
        if self.crane_state == self.sp.STOPPED:
            # ================================
            # DROPOFF
            # ================================
            if self.crane_weight.value_smooth < self.weight_threshold_empty_pot:
                # ================================
                # hot pots dropoff coming from the cascade
                # ================================
                if self.pot_state == self.sp.HOT and self.zone != 1:
                    # self.update_grid_model(None) # dropped off a hot pot at x,y
                    # TODO: there is no garbage collection mechanism in this kind of dict
                    self.hot_pot_dropoff[self.timestamp] = (self.crane_x.value,
                                                            self.crane_x_discrete.value,
                                                            self.crane_y.value,
                                                            self.hot_pot_cascade_pickup_info)
                    self.hot_pot_last_dropoff = (self.timestamp,
                                                 self.crane_x.value,
                                                 self.crane_x_discrete.value,
                                                 self.crane_y.value)
                    self.hot_pot_reposition_window = True # allow the pot to be repositioned

                    # set occupancy
                    [y,x] = self.gm.getGridCo(grid_model_parameters=self.gm.craneGridMappingParameters,
                                              y=self.crane_y.value,
                                              x=self.crane_x.value)
                    if [y,x] != [None,None]:
                        self.gm.occupancyMap.updatePosDeduction(y,x)
                        if self.debug: print(f'≡≡≡≡≡ |x| hot cascade pot dropped at [{x},{y}]')

                    # if there was another pot too close, delete that pot
                    timestamp_matches = CraneObserver.find_closest_pot(self.empty_pot_dropoff,
                                                                       self.crane_x.value,
                                                                       self.crane_y.value,
                                                                       self.pot_searching_distance*0.8,
                                                                       force_multiple=True)
                    for t in timestamp_matches:
                        del self.empty_pot_dropoff[t]

                # ================================
                # empty pot dropoff on the slagsquare
                # ================================
                if self.load_state == self.sp.EMPTY and self.zone == 0:
                    # closest pot snippet is added because crane movements with empty pots
                    # are a bit more aggressive than full pots
                    timestamp_matches = CraneObserver.find_closest_pot(self.empty_pot_dropoff,
                                                                       self.crane_x.value,
                                                                       self.crane_y.value,
                                                                       self.pot_searching_distance*0.6,
                                                                       force_multiple=True)
                    for t in timestamp_matches:
                        del self.empty_pot_dropoff[t]
                    self.empty_pot_dropoff[self.timestamp] = (self.crane_x.value,
                                                              self.crane_x_discrete.value,
                                                              self.crane_y.value,
                                                              self.pot_slagsquare_pickup_info)
                    self.empty_pot_last_dropoff = (self.timestamp,
                                                   self.crane_x.value,
                                                   self.crane_x_discrete.value,
                                                   self.crane_y.value)

                    [y,x] = self.gm.getGridCo(grid_model_parameters=self.gm.craneGridMappingParameters,
                                              y=self.crane_y.value,
                                              x=self.crane_x.value)
                    if [y,x] != [None,None]:
                        self.gm.occupancyMap.updatePosDeduction(y,x)
                        if self.debug: print(f'≡≡≡≡≡ |_| empty pot placed at [{x},{y}]')

                self.update_load_state_to(self.sp.NONE)
                self.update_pot_state_to(self.sp.UNKNOWN)

            # ================================
            # FILLED PICKUP
            # ================================
            if self.crane_weight.value_smooth > self.weight_threshold_filled_pot:
                self.update_load_state_to(self.sp.FULL)

                # ================================
                # hot pots coming from the cascade or repositioning in zone 0
                # ================================
                if self.pot_state != self.sp.HOT:
                    if self.zone == 1:
                        self.update_pot_state_to(self.sp.HOT)
                        self.hot_pot_cascade_pickup_info = (self.timestamp,
                                                            self.crane_x.value,
                                                            self.crane_y.value)
                    elif self.hot_pot_reposition_window:
                        self.update_pot_state_to(self.sp.HOT)
                        self.hot_pot_reposition_window = False
                         # delete last dropoff
                        last_timestamp = self.hot_pot_last_dropoff[0]
                        del self.hot_pot_dropoff[last_timestamp]
                        if self.debug: print(f'-------- removed hot pot dropoff {last_timestamp} (repositioning)')
                    elif self.zone == 0:
                        self.update_pot_state_to(self.sp.UNKNOWN)
                        self.pot_slagsquare_pickup_info = (self.timestamp,
                                                           self.crane_x.value,
                                                           self.crane_y.value)
                        # check if a full pot is in the hot pot collection
                        timestamp_matches = CraneObserver.find_closest_pot(self.hot_pot_dropoff,
                                                                           self.crane_x.value,
                                                                           self.crane_y.value,
                                                                           self.pot_searching_distance)
                        # if we can link a hot pot, delete this hot pot
                        for t in timestamp_matches:
                            # First update the occupancy map.
                            full_pot_x = self.hot_pot_dropoff[t][0]
                            full_pot_y = self.hot_pot_dropoff[t][2]
                            [y,x] = self.gm.getGridCo(grid_model_parameters=self.gm.craneGridMappingParameters,
                                                      y=full_pot_y,
                                                      x=full_pot_x)
                            if [y,x] != [None,None]:
                                self.gm.occupancyMap.updateNegDeduction(y,x)
                                if self.debug: print(f'≡x≡x≡ pot removed from [{x},{y}]')

                            del self.hot_pot_dropoff[t]

            # ================================
            # EMPTY PICKUP
            # ================================
            if self.crane_weight.value_smooth > self.weight_threshold_empty_pot and \
               self.crane_weight.value_smooth < self.weight_threshold_filled_pot:
                if self.load_state == self.sp.NONE:
                    if self.zone == 0:
                        timestamp_matches = CraneObserver.find_closest_pot(self.empty_pot_dropoff,
                                                                           self.crane_x.value,
                                                                           self.crane_y.value,
                                                                           self.pot_searching_distance)
                        for t in timestamp_matches:
                            # first update the occupancy map
                            empty_pot_x = self.empty_pot_dropoff[t][0]
                            empty_pot_y = self.empty_pot_dropoff[t][2]
                            [y,x] = self.gm.getGridCo(grid_model_parameters=self.gm.craneGridMappingParameters,
                                                      y=empty_pot_y,
                                                      x=empty_pot_x)
                            if [y,x] != [None,None]:
                                self.gm.occupancyMap.updateNegDeduction(y,x)
                                if self.debug: print(f'≡≡≡≡≡ empty pot taken from [{x},{y}]')

                            # This might actually delete multiple pots for empty pots,
                            # which is kind of welcome because the dropoff might have
                            # been jittery.
                            del self.empty_pot_dropoff[t]
                        self.update_load_state_to(self.sp.EMPTY)
                        if self.debug: print(f'≡≡≡≡≡ |_| picked up empty pot from zone 0')

        # ================================
        # EMPTYING OF FULL POTS
        # ================================
        # the crane can be moving or stopped
        # the pot weight is more than 0, but not very heavy
        if self.crane_weight.value_smooth > self.weight_threshold_empty_pot and \
           self.crane_weight.value_smooth < self.weight_threshold_filled_pot:
            # the previous state was FULL, NONE is added due to sensor noise
            if (self.load_state in [self.sp.NONE,self.sp.FULL]) and self.zone == 2:
                self.update_load_state_to(self.sp.EMPTY)
                if self.debug: print(f'--------emptied pot')


    @staticmethod
    def find_closest_pot(pot_collection,
                         x,y,
                         search_distance,
                         force_multiple=False,
                         debug=False):
        """Finds the closest pot to (x,y) in a collection of pots.

        It will return empty lists is no closest pots are found or if force_multiple
        is not enabled.

        Args:
        - pot_collection (dict):      the pot collection to find the closest pot in
        - x (float):                  x coordinate of the current pot
        - y (float):                  y coordinate of the current pot
        - search_distance (float):    minimal radius to look for a pot (center)
        - force_multiple (bool):      if True, it will return all close pots

        Returns:
        - list of timestamps that can be used as keys of the given collection
        """
        candidates = []
        for timestamp,data in pot_collection.items():
            x_ = data[0]
            y_ = data[2]
            distance = np.sqrt((x-x_)**2+(y-y_)**2)
            if distance < search_distance:
                candidates.append((timestamp,distance))
        if len(candidates) == 0:
            if debug: print('WARNING: No candidates found as close pots, returning []')
            return []
        elif len(candidates) >1:
            if force_multiple:
                if debug: print('INFO: Multiple close pots found, forcing return of all pots')
                return [c[0] for c in candidates] #return a list of timestamps
            else:
                if debug: print('WARNING: Multiple close pots found, returning []')
                return []
        else:
            assert len(candidates) == 1
            return [candidates[0][0]]


    def update_crane_state_to(self, new_state: CraneState):
        """Updates internal crane state.

        Args:
        - new_state (CraneState): new crane state.
        """
        if self.crane_state != new_state:
            self.crane_state = new_state
            if self.debug: print(f'changed crane state to {new_state.name}')


    def update_load_state_to(self, new_state: LoadState):
        """Updates internal load state.

        Args:
        - new_state (LoadState): new load state.
        """
        if self.load_state != new_state:
            self.load_state = new_state
            if self.debug: print(f'changed load state to {new_state.name}')


    def update_pot_state_to(self, new_state: PotState):
        """Updates internal pot state.

        Args:
        - new_state (PotState): new pot state.
        """
        if self.pot_state != new_state:
            self.pot_state = new_state
            if self.debug: print(f'changed pot state to {new_state.name}')


    @staticmethod
    def update_grid_model(grid_model: GridModel,
                          row: int,
                          col: int,
                          state: int):
        """Updates a grid model with state knowledge.

        Args:
        - grid_model (GridModel): the gridmodel to update
        - row (int):              row of the gridmodel
        - col (int):              column of the gridmodel
        - state (int):            occupancy state, 0=free, 1=occupied
        """
        grid_model.setOccupancy(row,col,state)


    def update_zone(self):
        """Sets zone id based on internal crane x,y readings."""
        # x, y, w, h
        zone_1 = [85.,0.,30.,16.] # cascade
        zone_2 = [ 0.,0.,30.,16.] # emptying station

        def check_zone(zone):
            if  self.crane_x.value > zone[0]           \
            and self.crane_x.value < zone[0]+zone[2]   \
            and self.crane_y.value > zone[1]           \
            and self.crane_y.value < zone[1]+zone[3]:
                return True
            else:
                return False

        if check_zone(zone_1):
            self.zone = 1
        elif check_zone(zone_2):
            self.zone = 2
        else:
            self.zone = 0
