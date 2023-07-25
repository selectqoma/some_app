""" slagsquare_pot_states_osi_pi_publisher - This module contains a class that
    can be used to publish pot states to the Umicore OSI PI Server
"""

import sys
import time
import json
from threading import Lock

import rospy
import message_filters as mf
from nav_msgs.msg import OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from smelter_osi_pi import osi_pi_utils
from smelter_msgs.msg import PotStateArray
from smelter_srvs.srv import JSON, JSONRequest


class PotStatesOSIPIPublisher:
    """ Receives pot states via ROS msg and publishes them to the Umicore
        OSI PI server
    """

    def __init__(self):
        """ Loads variables from the param server and initializes ros
            interfaces
        """
        self.tag_urls = None
        self.latest_input_timestamp = None
        self.latest_output_timestamp = None
        self.lock = Lock()
        self.load_params()
        self.initialize()


    def initialize(self):
        """ Performs initialization of variables and interfaces
        """
        self.initialize_diagnostics()
        self.initialize_proxy_interface()
        self.initialize_states()
        self.initialize_subscribers()


    def load_params(self):
        """ Loads osi pi parameters from the parameter server
        """
        # load webapi params
        self.piwebapi_url = rospy.get_param('~piwebapi_url')
        self.osi_af_server_name = rospy.get_param('~af_server_name')
        self.osi_af_database_name = rospy.get_param('~af_database_name')
        self.piwebapi_user = rospy.get_param('~piwebapi_user')
        self.piwebapi_pass = rospy.get_param('~piwebapi_pass')
        self.piwebapi_auth = rospy.get_param('~piwebapi_security_method')

        # load asset framework hierarchy params
        self.asset_server_pots_hierarchy = rospy.get_param(
            '~asset_server_pots_hierarchy')
        self.attributes_template = rospy.get_param('~attributes_template')
        self.pots_path = rospy.get_param('~pots_path')
        self.keizer_pots_path = rospy.get_param('~keizer_pots_path')

        self.attributes_config_prefix = \
            rospy.get_param('~attribute_config_prefix')

        self.pots_attribute_prefix = rospy.get_param('~pots_attribute_prefix')
        self.keizer_pots_attribute_prefix = rospy.get_param(
            '~keizer_pots_attribute_prefix')

        self.temperature_attribute_suffix = rospy.get_param(
            '~temperature_attribute_suffix')
        self.presense_attribute_suffix = rospy.get_param(
            '~presense_attribute_suffix')

        # config params
        self.rows = rospy.get_param('~rows')
        self.cols = rospy.get_param('~cols')
        self.presense_threshold = rospy.get_param('~presense_threshold')
        self.column_indexing_offset = rospy.get_param(
            '~column_indexing_offset')
        self.diagnostics_period = rospy.get_param('~diagnostics_period')
        self.max_update_period = rospy.get_param('~max_update_period')
        self.use_proxy = rospy.get_param('~use_proxy', False)


    def initialize_subscribers(self):
        """ Creates a synchronized subscription to the pot states and the
        occupancy grid plus a service to use osi pi proxy
        """
        # create subscribers for synchronization
        pot_states_sub = mf.Subscriber('pot_states', PotStateArray)
        ogm_sub = mf.Subscriber('occupancy_grid/map', OccupancyGrid)

        # create synchronizer
        self.ts = mf.TimeSynchronizer([pot_states_sub, ogm_sub], 10)

        # register the callback to the synchronizer
        self.ts.registerCallback(self.pot_states_callback)


    def initialize_proxy_interface(self):
        """ Initializes the interface to send requests to OSI PI through a
            proxy
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


    def initialize_diagnostics(self):
        """ Sets up the diagnostics for this node
        """
        # create diagnostics message publisher
        self.diagnostics_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=1)#, latch=True)

        # create timer to run the diagnostics every hour
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(self.diagnostics_period), self.diagnostics_callback)


    def initialize_states(self):
        """ Reads the tag ids and initializes the states of the pots
        """
        # initialize states dictionary
        self.states = {(x, y): {'temperature': -20, 'presense': 0}
                       for x in range(self.cols)
                       for y in range(self.rows)}

        # add invalid slots that belong to slakkentrein
        invalid_slots = [(x, 0) for x in range(23, 28)]

        # add invalid pots due to the keizer pots taking 1.5 slots
        invalid_slots += [(x, self.rows-1) for x in range(18, 23)]

        #  add invalid pots due to carousel
        invalid_slots += [(x, 0) for x in range(14, 18)]

        # set those slot states to None
        for slot in invalid_slots:
            del self.states[slot]

        # read the urls for the presense and temperature tags of each pot
        #  self.read_tag_urls()
        self.read_batch_tag_urls()

        # call diagnostics once to publish initial status
        rospy.sleep(1.0)
        self.diagnostics_callback(None)

        if self.num_found_tags == 0:
            rospy.logerr('Failed to read pot tag web ids')
            rospy.signal_shutdown('Failed to read pot tag web ids')
            sys.exit(0)
        else:
            rospy.logwarn(
                f'Found pot tags [{self.num_found_tags}/{self.num_tags}]')


    def read_batch_tag_urls(self):
        """ Reads and stores the URLs of the pot tags to write to in batch mode
        """
        rospy.loginfo('Reading pot states osi pi tags in batch mode')
        self.tag_urls = {}
        attributes_batch = {}

        for col, row in self.states:
            elements_seq, presense_attribute, temperature_attribute = \
                self.get_attribute_paths(row, col)

            attributes_batch[(col, row)] = {
                'elements_seq': elements_seq,
                'attributes': {
                    'presense': presense_attribute,
                    'temperature': temperature_attribute
                }
            }

        if self.use_proxy:
            request = osi_pi_utils.construct_batch_read_attribute_info(
                piwebapi_url=self.piwebapi_url,
                asset_server=self.osi_af_server_name,
                user_name=self.piwebapi_user,
                user_password=self.piwebapi_pass,
                piwebapi_security_method=self.piwebapi_auth,
                osi_af_database=self.osi_af_database_name,
                osi_af_attributes_batch=attributes_batch)
            serialized_request = osi_pi_utils.serialize_request(request)
            request_msg = JSONRequest(request=serialized_request)

            try:
                response = self.submit_request(request_msg)
            except rospy.ServiceException as exc:
                rospy.logerr(f'Service did not process request: {exc}')
                return False

            if response.success and response.response != '':
                response = response.response
            else:
                response = None

        else:
            response = osi_pi_utils.read_batch_attribute_info(
                piwebapi_url=self.piwebapi_url,
                asset_server=self.osi_af_server_name,
                user_name=self.piwebapi_user,
                user_password=self.piwebapi_pass,
                piwebapi_security_method=self.piwebapi_auth,
                osi_af_database=self.osi_af_database_name,
                osi_af_attributes_batch=attributes_batch)

            if response is not None and response.status_code == 207:
                response = response.text
            else:
                response = None

        if response is None:
            rospy.logerr('Failed to read pot state tag web ids')
            rospy.signal_shutdown('Failed to read pot state tag web ids')
            sys.exit(0)

        # deserialize json response
        data = json.loads(response)
        # rospy.logerr(data)

        for col, row in self.states:
            self.tag_urls[(col, row)] = {
                'presense': data[str(((col, row), 'presense'))]['Content']['WebId'],
                'temperature': data[str(((col, row), 'temperature'))]['Content']['WebId']}

        rospy.loginfo(
            f'Reading osi pi tags is done [{self.num_found_tags}'
            f'/{self.num_tags}]')

        return True


    def read_tag_urls(self):
        """ (DEPRECATED) Reads and stores the URLs of the pot tags to write to
            avoid reading them in every callback call
        """
        rospy.loginfo('Reading osi pi tags')
        self.tag_urls = {}

        # find the web id for each pot tag
        for col, row in self.states:
            self.tag_urls[(col, row)] = self.read_tag_urls_helper(row, col)

        rospy.loginfo(
            f'Reading osi pi tags is done [{self.num_found_tags}'
            f'/{self.num_tags}]')


    def read_tag_urls_helper(self, row, col):
        """ (DEPRECATED) Reads the temperature and presense URLs for a pot

            Keyword arguments:
            row -- The row that the pot is placed
            col -- The column that the pot is placed
        """
        elements_seq, presense_attribute, temperature_attribute = \
            self.get_attribute_paths(row, col)

        # request the tag web ids for the attributes
        urls = {}
        urls['presense'] = self.get_tag_write_helper(
            elements_seq, presense_attribute)
        urls['temperature'] = self.get_tag_write_helper(
            elements_seq, temperature_attribute)

        # return None in case of failure
        if urls['presense'] is None or urls['temperature'] is None:
            return None

        return urls


    def get_attribute_paths(self, row, col):
        """ Returns the element path to the presense and temperature attributes

            Keyword arguments:
            row -- The row that the pot is placed
            col -- The column that the pot is placed
        """
        # get element and attribute prefix names for specific pot
        leaf_element, attribute_prefix = self.get_pot_path_info(row, col)

        # construct path to element
        elements_seq = self.asset_server_pots_hierarchy + [leaf_element]

        # construct presense attribute path
        presense_attribute = self.attributes_template.format(
            attribute_prefix, self.presense_attribute_suffix)

        # construct temperature attribute path
        temperature_attribute = self.attributes_template.format(
            attribute_prefix, self.temperature_attribute_suffix)

        return elements_seq, presense_attribute, temperature_attribute


    def get_tag_write_helper(self, elements_seq, attribute):
        """ Wraps the get_tag_write_url function to reduce redundant code

            Keyword arguments:
            elements_seq -- A list with the sequence of elements before the
            attribute
            attribute -- The name of the attribute
        """
        return osi_pi_utils.get_tag_write_url(
            piwebapi_url=self.piwebapi_url,
            asset_server=self.osi_af_server_name,
            user_name=self.piwebapi_user,
            user_password=self.piwebapi_pass,
            piwebapi_security_method=self.piwebapi_auth,
            osi_af_database=self.osi_af_database_name,
            osi_af_element_path=elements_seq,
            osi_af_attribute_tag=attribute)


    def pot_states_callback(self, pot_states, ogm):
        """ Callback that receives the pot states

            Keyword arguments:
            msg -- The pot states message for updating the visualization
        """
        rospy.loginfo('Sending pot states to OSI PI')
        states = self.states.copy()
        self.latest_input_timestamp = rospy.Time.now()

        # get the temperatures of all detected pots of latest camera view
        for state in pot_states.pot_states:
            x = int(state.position.x) + self.column_indexing_offset
            y = int(state.position.y)

            # if pot is invalid skip it
            if (x,y) not in states or states[(x, y)] is None:
                continue

            # update temperature
            temperature = state.temperature_history[0].temperature
            states[(x, y)]['temperature'] = temperature

        # get the presense probability in each pot slot
        for x in range(self.column_indexing_offset, self.cols):
            for y in range(self.rows):

                # exclude invalid pots
                if (x, y) not in states or states[(x, y)] is None:
                    continue

                #  calculate indices in occupancy grid
                ogm_x = x - self.column_indexing_offset
                ogm_cols = self.cols - self.column_indexing_offset

                # find presense probability
                p = ogm.data[y * ogm_cols + ogm_x] / 100.0

                # clear presense if invalid temperature
                if states[(x, y)]['temperature'] > -20:
                    states[(x, y)]['presense'] = \
                        int(p > self.presense_threshold)
                else:
                    states[(x, y)]['presense'] = 0

                '''
                Clear temperature if uncertain presence is disabled because we
                don't want to change the temperature with a magic value in case
                we have faulty detection
                '''
                # if states[(x, y)]['presense'] == 0:
                    # states[(x, y)]['temperature'] = -20

        # publish the states to OSI PI
        self.write_pot_states(states)
        self.latest_output_timestamp = rospy.Time.now()
        rospy.loginfo('New pot states were sent to OSI PI')

        # update states
        with self.lock:
            self.states = states.copy()


    def write_batch_pot_states(self, states):
        """ Publishes in batch the pot states to OSI PI using the PI Web API

            Keyword arguments:
            pot_states: the pot states to write to the OSI PI server
        """

        # initialize batch request
        batch_request = {}

        # add requests to batch request
        for state in states.items():
            if state[1] is None:
                continue

            # extract indices from state
            try:
                row, col = self.get_position_from_state(state)
            except ValueError as e:
                rospy.logwarn(f'Extracting indices from state failed with: {e}')
                return False

            if (col, row) not in self.tag_urls or self.tag_urls[(col, row)] is None:
                rospy.logerr(f'Tags not found for pot {row}, {col})')
                continue

            # get data values to publish
            temperature, presense = self.get_data_from_state(state)

            # add request for temperature
            batch_request[str((row, col, 'temperature'))] = {
                'Method': 'POST',
                'Resource': self.piwebapi_url + '/streams/' + \
                        self.tag_urls[(col, row)]['temperature'] + '/recorded',
                'Content': '[{\'Value\': \'' + str(temperature) + '\'}]'
            }

            # add request for presense
            batch_request[str((row, col, 'presense'))] = {
                'Method': 'POST',
                'Resource': self.piwebapi_url + '/streams/' + \
                        self.tag_urls[(col, row)]['presense'] + '/recorded',
                'Content': '[{\'Value\': \'' + str(presense) + '\'}]'
            }

        # send batch request
        rospy.loginfo('Writing pot states via batch request')

        if self.use_proxy:
            request = osi_pi_utils.construct_batch_request(
                self.piwebapi_url, self.piwebapi_user, self.piwebapi_pass,
                self.piwebapi_auth, batch_request)
            serialized_request = osi_pi_utils.serialize_request(request)
            request_msg = JSONRequest(request=serialized_request)

            try:
                response = self.submit_request(request_msg)
                ret = response.success
            except rospy.ServiceException as exc:
                rospy.logerr(f'Service did not process request: {exc}')
                ret = False
        else:
            ret = osi_pi_utils.write_batch_request(
                piwebapi_url=self.piwebapi_url,
                user_name=self.piwebapi_user,
                user_password=self.piwebapi_pass,
                piwebapi_security_method=self.piwebapi_auth,
                batch_request=batch_request)

        rospy.loginfo('Pot states batch write request finished with: '
                      f'success={ret}')

        return ret


    def write_pot_states(self, states):
        """ Publishes the pot states to OSI PI using the PI Web API

            Keyword arguments:
            pot_states: the pot states to write to the OSI PI server
        """
        result = True

        self.write_batch_pot_states(states)
        #  for s in states.items():
            #  if s[1] is not None:
                #  result *= self.write_pot_state(s)

        return result


    def write_pot_state(self, state):
        """ (DEPRECATED) Writes temperature and presense of a Pot to the OSI PI
            server

            Keyword arguments:
            state -- The pot state to publish to the server. Should be in the
            form tuple((row, column), {'temperature': t, 'presense': p})
        """
        # extract indices from state
        try:
            row, col = self.get_position_from_state(state)
        except ValueError as e:
            rospy.logwarn(f'Extracting indices from state failed with: {e}')
            return False

        # get data values to publish
        temperature, presense = self.get_data_from_state(state)

        # get element and attribute prefix names for specific pot
        leaf_element, attribute_prefix = self.get_pot_path_info(row, col)

        # construct path to element
        elements_seq = self.asset_server_pots_hierarchy + [leaf_element]

        # construct name of temperature attribute
        temperature_attribute_name = self.attributes_template.format(
            attribute_prefix,
            self.temperature_attribute_suffix)

        # construct name of presense attribute
        presense_attribute_name = self.attributes_template.format(
            attribute_prefix,
            self.presense_attribute_suffix)

        # write to temperature tag
        t_ret = self.write_single_value(
            osi_af_element_path=elements_seq,
            osi_af_attribute_tag=temperature_attribute_name,
            data_value=temperature)

        # write to presense tag
        p_ret = self.write_single_value(
            osi_af_element_path=elements_seq,
            osi_af_attribute_tag=presense_attribute_name,
            data_value=presense)

        if not t_ret:
            rospy.logerr('Failed to write temperature for pot '
                         f'{"ABCDEFGH"[row]}{col+1} to the OSI PI Server')
        if not p_ret:
            rospy.logerr('Failed to write presense for pot '
                         f'{"ABCDEFGH"[row]}{col+1} to the OSI PI Server')

        return t_ret and p_ret


    def write_single_value(self, osi_af_element_path, osi_af_attribute_tag,
                           data_value):
        """ Wraps the function from osi_pi_utils to reduce code repetition

            Keyword arguments:
            osi_af_element -- The path to an element to write to
            osi_af_attrubute -- The attribute of an element to write to
            data_value -- The data value to write to the element's attribute
        """
        return osi_pi_utils.write_single_value(
            piwebapi_url=self.piwebapi_url,
            asset_server=self.osi_af_server_name,
            user_name=self.piwebapi_user,
            user_password=self.piwebapi_pass,
            piwebapi_security_method=self.piwebapi_auth,
            osi_af_database=self.osi_af_database_name,
            osi_af_element_path=osi_af_element_path,
            osi_af_attribute_tag=osi_af_attribute_tag,
            data_value=data_value)


    def get_position_from_state(self, state):
        """ Returns the alphanumeric row and numeric col based on indices

            Keyword arguments:
            state -- A tuple containing the row and column indices

            Details:
            - The value of row is in the range 0-7 and it's mapped to
            alphanumeric chars A-H.
            - The value of column is in the range 0-19 and it's mapped to the
            range 6-25 to correspond with the slagsquare layout. For the last
            row the cols are not offsetted because Keizer pots have a different
            mapping starting from 1 and indexed from 0
        """
        row = state[0][1]
        col = state[0][0]

        if row < 0 or row > self.rows - 1:
            raise ValueError(f'row is {row} but the valid range is '
                             f'[0, {self.rows}]')
        elif col < 0 or col > self.cols - 1:
            raise ValueError(f'col is {col} but the valid range is '
                             f'[0, {self.cols}]')
        elif row == 7 and 18 <= col <= 22:
            raise ValueError(f'Tried to calculate indices for pot H{col} but '
                             f'columns 18 to 22 are invalid for row H, {state}')

        return row, col


    def get_pot_path_info(self, row, col):
        """ Returns the leaf_element for the pot in the given row and column

            Keyword arguments:
            row -- The row of the pot
            col -- the column of the pot
        """
        # check if keizer pot and return pot info accordingly
        if row == 7 and 8 <= col <= 17:
            leaf_element = self.keizer_pots_path
            kcol = col - 8 + 1
            attribute_prefix = self.keizer_pots_attribute_prefix.format(kcol)
        else:
            row_char = self.get_alphabet_char_from_index(row)
            leaf_element = self.pots_path.format(row_char)
            attribute_prefix = self.pots_attribute_prefix.format(
                    row_char, col+1)

        return leaf_element, attribute_prefix


    def get_alphabet_char_from_index(self, index):
        """ Returns an alphanumeric character based on index

            Keyword arguments:
            index -- Return the alphabet character corresponding to the index
        """
        if index in range(26):
            return chr(65+index)

        raise ValueError


    def get_data_from_state(self, state):
        """ Returns temperature and presense values from status

            Keyword arguments:
            state -- The state to extract temperature and presense from
        """
        temperature = state[1]['temperature']
        presense = int(state[1]['presense'])
        return temperature, presense


    def diagnostics_callback(self, event):
        """ Publishes diagnostics about writing to OSI PI

            Keyword arguments:
            event -- The timer event (not used)
        """
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        msg.status.append(self.get_tags_diagnostic_status())
        msg.status.append(self.get_tag_updates_diagnostic_status())
        self.diagnostics_pub.publish(msg)


    def get_tags_diagnostic_status(self):
        """ Returns a status based on if all the tag ids were read successfully
        """
        status = DiagnosticStatus()
        status.name = rospy.get_namespace()[1:] + 'osi_pi/pot_states/tag_web_ids'
        status.hardware_id = ''

        if self.num_tags == self.num_found_tags:
            status.level = DiagnosticStatus.OK
            status.message = 'All tags found succesfully'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Not all tags were found successfully'

        status.values.append(KeyValue(
            key='num_total_tags', value=str(self.num_tags)))
        status.values.append(KeyValue(
            key='num_found_tags', value=str(self.num_found_tags)))
        status.values.append(KeyValue(
            key='found_tags_percentage',
            value=str(self.found_tags_percentage)))

        return status


    def get_tag_updates_diagnostic_status(self):
        """ Returns a status based on whether there have been tag updates
        """
        status = DiagnosticStatus()
        status.name = rospy.get_namespace()[1:] + 'osi_pi/pot_states/tag_updates'
        status.hardware_id = ''

        if self.latest_input_timestamp is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'No pot states received yet'
            return status

        try:
            input_is_stale = self.is_stale(
                self.latest_input_timestamp, self.max_update_period)
        except ValueError as e:
            rospy.logwarn(f'Checking latest input for stale status failed with: {e}')
            input_is_stale = True

        try:
            output_is_stale = self.is_stale(
                self.latest_output_timestamp, self.max_update_period)
        except ValueError as e:
            rospy.logwarn(f'Checking latest output for stale status failed with: {e}')
            output_is_stale = True

        if not input_is_stale and not output_is_stale:
           status.level = DiagnosticStatus.OK
           status.message = 'Input and output within expected update rate'
        elif input_is_stale and not output_is_stale:
           status.level = DiagnosticStatus.ERROR
           status.message = 'Haven\'t received any pot states in' + \
               f'{self.get_elapsed_time(self.latest_input_timestamp)} seconds'
        elif input_is_stale and not output_is_stale:
           status.level = DiagnosticStatus.ERROR
           status.message = 'Haven\'t published received pot states in' + \
               f'{self.get_elapsed_time(self.latest_input_timestamp)} seconds'
        else:
           status.level = DiagnosticStatus.WARN
           status.message = 'Haven\'t received any pot states yet'

        status.values.append(KeyValue(
            key='latest_pot_states_stamp',
            value=str(self.latest_input_timestamp)))
        status.values.append(KeyValue(
            key='latest_pot_states_publish_stamp',
            value=str(self.latest_output_timestamp)))

        return status


    @property
    def num_tags(self):
        """ Returns the total number of tags
        """
        return sum([self.states[key] is not None for key in self.states])


    @property
    def num_found_tags(self):
        """ Returns the total number of found tags
        """
        return min(self.num_tags, sum([self.tag_urls[key] is not None for key in self.tag_urls]))


    @property
    def found_tags_percentage(self):
        """ Returns the percentage of found tags
        """
        return int(min(self.num_found_tags, self.num_tags) / self.num_tags * 100)


    def get_elapsed_time(self, timestamp):
        """ Get the elapsed time since the given timestamp

            Keyword arguments:
            timestamp -- The timestamp to use for calculating elapsed time
        """
        if isinstance(timestamp, float):
            return time.time() - timestamp
        elif isinstance(timestamp, rospy.Time):
            return time.time() - timestamp.to_sec()
        else:
            raise ValueError('Given timestamp must be float or rospy.Time ' \
                             f'instead of {type(timestamp)}')

    def is_stale(self, timestamp, threshold):
        """ Returns True if the elapsed time since the timestamp is above the
            given threshold

            Keyword arguments:
            timestamp -- The timestmap ot check for stale status
            threshold -- The duration threshold for the stale status
        """
        if not isinstance(timestamp, float) and not \
                isinstance(timestamp, rospy.Time):
            raise ValueError('Given timestamp must be float or rospy.Time ' \
                             f'instead of {type(timestamp)}')

        return self.get_elapsed_time(timestamp) > threshold
