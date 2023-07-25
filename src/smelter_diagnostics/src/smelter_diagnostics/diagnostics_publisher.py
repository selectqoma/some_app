""" diagnostics_publisher.py -- This module contains the DiagnosticsPublisher
    class that receives aggregated diagnostics from the smelter applications
    and sends them to OSI PI
"""

import sys
import json
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from smelter_srvs.srv import JSON, JSONRequest
from smelter_osi_pi import osi_pi_utils


# converts between diagnostics status to
DIAGNOSTICS_TO_OSI_PI_STATUS_MAP = {
    DiagnosticStatus.STALE: 0,
    DiagnosticStatus.ERROR: 1,
    DiagnosticStatus.WARN: 2,
    DiagnosticStatus.OK: 3,
    'Stale': 0,
    'Error': 1,
    'Warn': 2,
    'ok': 3,
    'STALE': 0,
    'ERROR': 1,
    'WARN': 2,
    'OK': 3
}


class DiagnosticsPublisher:
    """ This class receives aggregated diagnostics from the smelter
        applications and sends them to OSI PI via the OSI PI proxy node
    """

    def __init__(self):
        """ Initializes subscribers and publishers
        """
        # load params from parameter server
        self.load_params()

        # initialize osi pi proxy interface through ROS services
        self.init_proxy_interface()

        # read the web ids to construct the urls of the tags
        rospy.loginfo('Reading diagnostic tag web ids')
        self.tag_urls = self.read_tag_urls()

        # create subscriber to aggregated dianostic messages
        rospy.Subscriber(
            '/diagnostics_agg', DiagnosticArray, self.callback, queue_size=1)


    def load_params(self):
        """ Loads parameters from the ROS param server
        """
        self.use_proxy = rospy.get_param('~use_proxy', False)

        # load osi pi parameters
        self.piwebapi_url = rospy.get_param('~piwebapi_url')
        self.osi_af_server_name = rospy.get_param('~af_server_name')
        self.osi_af_database_name = rospy.get_param('~af_database_name')
        self.piwebapi_user = rospy.get_param('~piwebapi_user')
        self.piwebapi_pass = rospy.get_param('~piwebapi_pass')
        self.piwebapi_auth = rospy.get_param('~piwebapi_security_method')
        self.hierarchy = rospy.get_param('~asset_server_hierarchy')
        self.tags = rospy.get_param('~tags')


    def init_proxy_interface(self):
        """ Initializes the ROS services interace to communicate with the
            OSI PI proxy node
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


    def read_tag_urls(self):
        """ Reads the web ids of the tags from OSI PI
        """
        tag_urls = {}
        attributes_batch = {}

        for category in self.tags:
            for tag_key in self.tags[category]:
                attributes_batch[tag_key] = {
                    'elements_seq': self.hierarchy,
                    'attributes': {
                        'status': self.tags[category][tag_key]
                        }
                }

        # request tag urls
        if self.use_proxy:
            request = osi_pi_utils.construct_batch_read_attribute_info(
                self.piwebapi_url, self.osi_af_server_name, self.piwebapi_user,
                self.piwebapi_pass, self.piwebapi_auth, self.osi_af_database_name,
                attributes_batch)
            serialized_request = osi_pi_utils.serialize_request(request)
            request_msg = JSONRequest(request=serialized_request)

            try:
                response = self.submit_request(request_msg)
            except rospy.ServiceException as exc:
                rospy.logerr(f'Service did not process request: {exc}')
                return None

            if response.success and response.response != '':
                response = response.response
            else:
                response = None
        else:
            response = osi_pi_utils.read_batch_attribute_info(
                self.piwebapi_url, self.osi_af_server_name,
                self.piwebapi_user, self.piwebapi_pass, self.piwebapi_auth,
                self.osi_af_database_name, attributes_batch)

            if response is not None and response.status_code == 207:
                response = response.text
            else:
                response = None

        if response is None:
            rospy.logerr('Failed to read diagnostic tag web ids')
            rospy.signal_shutdown('Failed to read diagnostic tag web ids')
            sys.exit(0)

        data = json.loads(response)

        for category in self.tags:
            tag_urls[category] = {}
            for tag_key in self.tags[category]:
                tag_urls[category][tag_key] = \
                        data[str((tag_key, 'status'))]['Content']['WebId']

        return tag_urls


    def callback(self, msg):
        """ Receives an aggregated diagnostics message and publishes it to
            OSI PI through the OSI PI proxy node

            Parameters:
            msg (DiagnosticArray): Contains diagnostics info about the apps
        """
        # Construct batch write request
        tag_values = self.match_diagnostics_with_tags(msg)
        batch_request = {}
        for idx, tag in enumerate(tag_values):
            batch_request[idx] = {
                'Method': 'POST',
                'Resource': self.piwebapi_url + '/streams/' + tag + '/recorded',
                'Content': '[{\'Value\': \'' + str(tag_values[tag]) +'\'}]'
            }

        # Send request to OSI PI either through proxy or directly
        rospy.loginfo('Publishing diagnostics to OSI PI')
        if self.use_proxy:
            request = osi_pi_utils.construct_batch_request(
                self.piwebapi_url, self.piwebapi_user, self.piwebapi_pass,
                self.piwebapi_auth, batch_request)
            serialized_request = osi_pi_utils.serialize_request(request)
            request_msg = JSONRequest(request=serialized_request)

            try:
                response = self.submit_request(request_msg)
            except rospy.ServiceException as exc:
                rospy.logerr(f'Service did not process request: {exc}')
                return

            if not response.success:
                rospy.logerr(f'Publishing diagnostics status failed with '
                             f'response: {response.response}')
        else:
            ret = osi_pi_utils.write_batch_request(
                piwebapi_url=self.piwebapi_url,
                user_name=self.piwebapi_user,
                user_password=self.piwebapi_pass,
                piwebapi_security_method=self.piwebapi_auth,
                batch_request=batch_request)

            if not ret:
                rospy.logerr('Diagnostics batch request failed with: '
                             f'{response.reason}')


    def match_diagnostics_with_tags(self, msg):
        """ Processes an aggregated diagnostics msg and returns a states dict
            The dictionary contains osi_pi tag - value pairs

            Parameters:
            msg(DiagnosticArray): Contains the aggregated diagnostics
        """
        states = {}
        for status in msg.status:
            try:
                category, module = status.name[1:].split('/')[:2]
            except ValueError as exc:
                continue
            if category in self.tag_urls and module in self.tag_urls[category]:
                states[self.tag_urls[category][module]] = \
                        DIAGNOSTICS_TO_OSI_PI_STATUS_MAP[status.level]

        return states
