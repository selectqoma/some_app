import json
import datetime
import time
import requests
from requests.auth import HTTPBasicAuth
import pandas as pd
import numpy as np
import pickle

from smelter_osi_pi import osi_pi_utils

try:
    import rospy
    from smelter_srvs.srv import JSON, JSONRequest
    USE_PROXY=True
except ImportError:
    USE_PROXY = False


class CraneReader:
    """
    The CraneReader class extracts and prepares data from the OSI Pi server.

    Because multiple tags need to be pulled and we don't want to overload
    the Pi network, the crane reader pulls tags in the following manner:

    Time    Action      Description
    --------------------------------------------------
    00:00   init        pull set of tags of previous 1m
    00:01   update      pull set of tags of previous 1m
    00:02   update      pull set of tags of previous 1m
    00:05   update      pull set of tags of previous 1m

    Parameters:
    - config (dict):        dict that contains Pi server configuration
    - web_ids (dict):       dict of tags and corresponding web_ids
    - dump_file (string):   path to optional dump file for offline use
    """
    def __init__(self, config, web_ids, use_proxy=False, dump_file=None):
        self.last_update = None

        self.config = config
        self.web_ids = web_ids

        self.tags = {}

        self.dump_file = dump_file

        global USE_PROXY
        self.use_proxy = use_proxy and USE_PROXY

        self.initialize_proxy_interface()


    @staticmethod
    def get_current_minute():
        """Returns the last (current) minute without seconds."""
        #  dt = datetime.datetime.utcnow()
        dt = datetime.datetime.fromtimestamp(rospy.Time.now().to_sec())
        dt_floor_minute = dt.strftime('%Y-%m-%d %H:%M')
        return datetime.datetime.strptime(dt_floor_minute, '%Y-%m-%d %H:%M')


    def initialize_proxy_interface(self):
        """ Initializes the interface to send requests to OSI PI through proxy
        """
        if self.use_proxy:
            try:
                rospy.wait_for_service('/osi_pi/submit_request', timeout=5)
                self.submit_request = rospy.ServiceProxy(
                    '/osi_pi/submit_request', JSON)
            except rospy.ROSException as exc:
                rospy.logwarn('Waiting for osi pi proxy service failed with: '
                              f'{exc}')
                self.use_proxy = False


    def read_from_stream_multiple_recorded(self, web_ids: dict, lookback_min=1):
        """
        Returns a collection of recorded stream values over a given time window.

        This functions should be preferred over direct value reading as it puts
        less stress on the network.

        Parameters:
        - web_ids (dict):       dictionary of webids
        - lookback_min (int):   update frequency in minutes

        Returns:
        - response (dict): request response in json format
        """
        timedelta = datetime.timedelta(minutes=lookback_min)
        end_time = CraneReader.get_current_minute()
        start_time = end_time - timedelta
        web_ids = list(web_ids.values())

        response = None

        if self.use_proxy:
            request = osi_pi_utils.construct_streamset_read_request(
                piwebapi_url=self.config['PIWEBAPI_URL'],
                user_name=self.config['USER_NAME'],
                user_password=self.config['USER_PASSWORD'],
                piwebapi_security_method='basic',
                web_ids=web_ids,
                start_time=start_time,
                end_time=end_time)

            serialized_request = osi_pi_utils.serialize_request(request)
            request_msg = JSONRequest(request=serialized_request)

            try:
                srv_response = self.submit_request(request_msg)
            except rospy.ServiceException as exc:
                rospy.logerr(f'Service did not process request: {exc}')
                return None

            # convert string to json
            if srv_response.success:
                response = json.loads(srv_response.response)
        else:
            response = osi_pi_utils.send_streamset_read_request(
                piwebapi_url=self.config['PIWEBAPI_URL'],
                user_name=self.config['USER_NAME'],
                user_password=self.config['USER_PASSWORD'],
                piwebapi_security_method='basic',
                web_ids=web_ids,
                start_time=start_time,
                end_time=end_time).json()

        return response


    def info(self):
        """Prints current tag values."""
        print(self.last_update)
        for k,v in self.tags.items():
            print(f'{k} : {v}')


    def dump_info(self):
        """Appends current tag values to the configured dump file."""
        if self.dump_file is None:
            print('Failed to dump info because no output dump file was '
                  'specified during initialization')
            return

        vals = [f'{v}' for _,v in self.tags.items()]
        write_line = f'{self.last_update};{";".join(vals)}\n'

        with open(self.dump_file, "a+") as myfile:
            myfile.write(write_line)


    def timed_generator(self):
        """
        Yields a response if enough time has passed (>1min).

        Yields:
        - series (dict):        dict of Series
        - timestamp (DateTime): timestamp of latest update
        """
        while True:
            # update timekeeping
            last_minute = CraneReader.get_current_minute()

            if self.last_update is None or self.last_update != last_minute:
                #  self.last_update = last_minute
                # TODO: there is no mechanism when the response fails
                response = self.read_from_stream_multiple_recorded(self.web_ids)

                # TODO(raphael): review this
                if response is None:
                    time.sleep(10)
                    continue

                self.last_update = last_minute

                series = {}
                for r in response['Items']:
                    timestamp = [i['Timestamp'] for i in r['Items']]
                    values = [i['Value'] for i in r['Items']]

                    ds = pd.Series(data=values,index=timestamp).copy()
                    ds = pd.to_numeric(ds, errors='coerce')

                    ds.index = pd.to_datetime(ds.index)
                    if len(ds) == 0:
                        ds.index = ds.index.tz_localize('GMT')

                    #series[r['WebId']] = ds
                    name_via_webId = {v:k for k,v in self.web_ids.items()}[r['WebId']]
                    series[name_via_webId] = ds
                    #rospy.logerr(f'r=\n{r}')

                yield (series,self.last_update.strftime('%Y-%m-%d %H:%M'))
            else:
                print( '| Trying to yield an update, but the last update was too recent')
                print(f'| last update: {self.last_update}, current last minute: {last_minute}')
                print(r'|\ sleeping 10 seconds')
                time.sleep(10)


    def osi_pi_buffer(self, pi_server):
        """
        Buffer fuction that processes individual value series into continuous pandas DataFrame.

        Recorded data coming from the Osi Pi server or the mock server can be sparse
        or contain a lot of nans. This function creates an intermediate buffer such
        that the data comes in a easier-to-work-with format. More specifically, the data
        will be served on a per-minute base and tags will be chopped into values per second.

        Mainly, the function uses the pandas interpolation function. But, before this
        interpolation becomes reliable, we need to construct some scaffolding
        around our timeslice of interest. For example, if we want to have all tag values
        in 16:44, we need to add 16:43 and 16:45 to ensure continuity.

        Parameters:
        - pi_server (generator): generator that returns osi pi data

        Yields:
        - window (DataFrame): dataframe of values
        """

        def transform_to_df(series,timestamp,latest_values_non_nan):
            df = pd.DataFrame(series)
            if len(df) == 0:
                print("WARNING transformed df length 0")
                df.loc[timestamp+'+00:00'] = latest_values_non_nan
                df.index = pd.to_datetime(df.index)
            return df

        latest_values_non_nan = np.zeros(5) # TODO: change hardcoded columns to amount of webids

        response_body,timestamp_body = next(pi_server)
        print("** buffer init: body done")
        df_body = transform_to_df(response_body,timestamp_body,latest_values_non_nan)

        response_tail,timestamp_tail = next(pi_server)
        print("** buffer init: tail done")
        df_tail = transform_to_df(response_tail,timestamp_tail,latest_values_non_nan)

        while True:
            df_head = df_body.copy()
            timestamp_head = timestamp_body
            df_body = df_tail.copy()
            timestamp_body = timestamp_tail
            response_tail,timestamp_tail = next(pi_server)
            print("----buffer iter done")
            df_tail = transform_to_df(response_tail,timestamp_tail,latest_values_non_nan)

            # Combine scaffolding.
            window = pd.concat([df_head,df_body,df_tail])

            # This is the core action of this function.
            window = window.resample('1S').mean().interpolate()

            # Record fresh values.
            new_latest_values_non_nan = window.iloc[-1]
            latest_values_non_nan[new_latest_values_non_nan.notna()] = new_latest_values_non_nan

            yield window[timestamp_head]


    def osi_pi_mock_generator(self, update_freq_min=1):
        """
        Generates mock streamset data.

        Parameters:
        - update_freq_min (int): update frequency in minutes

        Yields:
        - streamset (dict): dict of streamset Series
        - mark (DateTime):  timestamp of mock data point
        """
        hist = pickle.load(open(self.dump_file,"rb"))
        series = {}
        # Load data as independent series, this is also more or less how you will receive
        # the data from the osi pi server.
        for k,v in hist.items():
            timestamp = [i['Timestamp'] for i in v]
            values = [i['Value'] for i in v]
            if type(values[0]) == dict:
                values = [i['Value'] for i in values]

            # do some cleanup
            ds = pd.Series(data=values,index=timestamp).copy()
            if type(values[0]) == float:
                ds = pd.to_numeric(ds, errors='coerce')
            ds.index = pd.to_datetime(ds.index)
            series[k] = ds

        # Setting a start moment.
        # You can tweak this if you want to jump to interesting events.
        start = datetime.datetime.fromisoformat('2020-06-01 16:40+00:00') # note tz info
        # fromisoformat will not work on the kpv server as the function does not exist for python < 3.7
        update_delta = datetime.timedelta(minutes=update_freq_min)

        # provide the data each update_freq_min
        while True:
            timeslice = slice(start,start+update_delta)
            streamset = {}
            for k,v in series.items():
                streamset[k] = series[k][timeslice]
            mark = start.strftime('%Y-%m-%d %H:%M')
            start = start+update_delta

            yield (streamset,mark)


    def timed_mock_generator(self):
        """
        Yields a response if enough time has passed (>1min).

        Yields:
        - series (dict):        dict of Series
        - timestamp (DateTime): timestamp of latest update
        """

        # load the dataframe from the pickle
        hh = pickle.load(open(self.dump_file,"rb"))
        hh.iloc[0] = .0
        hh = hh.resample('1S').mean().interpolate().dropna()

        time_offset =  datetime.timedelta(hours=1)

        while True:
            # update timekeeping
            last_minute = CraneReader.get_current_minute()+time_offset

            if self.last_update is None or self.last_update != last_minute:

                # update timestamp of last update
                self.last_update = last_minute+time_offset

                yield hh[last_minute.strftime('%Y-%m-%d %H:%M')]
            else:
                print( '| Trying to yield an update, but the last update was too recent')
                print(f'| last update: {self.last_update}, current last minute: {last_minute}')
                print(r'|\ sleeping 10 seconds')
                time.sleep(10)
