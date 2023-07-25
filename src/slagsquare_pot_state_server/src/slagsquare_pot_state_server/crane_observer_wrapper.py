""" crane_observer_wrapper.py -- This file contains a class that wraps the
CraneObserver and CraneReader classes for easier seamless integration to the
pot state server
"""

import pandas as pd

from .crane_observer import CraneObserver
from .crane_reader import CraneReader


class CraneObserverWrapper:
    """ This class implements a wrapper of the CraneObserver and CraneReder

        Reads tags from OSI PI every minute and updates the grid model

        Attributes:

    """

    def __init__(self,
                 name: str,
                 grid_models: list,
                 config: dict
                 ) -> None:
        """ Initializes the crane observer and reader

        Args:
            name (str): The name of the crane
            grid_models (list): A list of grid models that the crane moves in
            config (dict): Contains config parameters
            use_proxy (bool): If true go through osi pi proxy node
        """

        # create crane observer instance
        self.crane_observer = CraneObserver(
            name=name,
            gridmodels=grid_models,
            parameter_mapping=config['cranes'][name]['mapping'])

        # create crane reader instance
        self.crane_reader = CraneReader(
            config=config['osi_pi'],
            web_ids=config['cranes'][name]['tags'],
            use_proxy=config['use_proxy'],
            dump_file=config['crane_filepath'])

        # create a timed generator
        if config['mock']:
            self.crane_buffer = \
                self.crane_reader.timed_mock_generator()
        else:
            self.crane_timed_generator = self.crane_reader.timed_generator()
            # create a crane buffer
            self.crane_buffer = self.crane_reader.osi_pi_buffer(
                self.crane_timed_generator)


    def read(self) -> pd.DataFrame:
        """ Reads tags from OSI PI and returns an interpolated pandas dataframe

            Returns
                pd.Dataframe: Contains the parameter history of the crane
        """

        crane_df = None

        try:
            crane_df = next(self.crane_buffer)
        except Exception as exc:
            print(f'Crane reading failed with exception: {exc}')

        return crane_df


    def update(self, crane_df) -> None:
        """ Processes the crane updates to update the grid models

            Args:
                crane_df (pd.DataFrame): The crane updates
        """
        for row in crane_df.iterrows():
            self.crane_observer.process(row)
        
        # uncomment the line below if you want to print occupancy maps
        # self.crane_observer.gm.occupancyMap.print_maps()