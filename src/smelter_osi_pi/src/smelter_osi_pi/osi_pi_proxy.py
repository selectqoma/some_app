""" osi_pi_proxy.py - This module contains a class that implements a proxy for
communicating with an OSI PI server ensuring mutual exclusion of concurrent
requests
"""

import os
import pickle
import time
from ast import literal_eval

import requests
import rospy
import urllib3
from smelter_srvs.srv import JSON, JSONResponse

from .osi_pi_utils import (call_security_method, pretty_print_request,
                           pretty_print_response)

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


class OSIPIProxy:
    """
    A proxy for an OSI PI server that handles read and write requests from
    numerous clients
    """

    def __init__(self):
        """ Initializes member variables and starts interfaces """
        self.load_params()
        self.reset()

    def reset(self):
        """ Initializes or resets variables and interfaces """
        self.initialize_osi_pi_interface()
        self.initialize_ros_interface()

    def load_params(self):
        """ Loads the clients parameters from the ros parameter server """
        self.piwebapi_url = rospy.get_param('~piwebapi_url')
        self.af_server_name = rospy.get_param('~af_server_name')
        self.username = rospy.get_param('~piwebapi_user')
        self.password = rospy.get_param('~piwebapi_pass')
        self.auth_type = rospy.get_param('~piwebapi_security_method')

    def initialize_ros_interface(self):
        """ Initializes ros publishers and subscribers """
        self.srv = rospy.Service('osi_pi/submit_request', JSON, self.submit_request_callback)

    def initialize_osi_pi_interface(self):
        """ Initializes the web client session """
        self.authentication = call_security_method(self.auth_type,
                                                   self.username,
                                                   self.password)
        self.session = requests.Session()
        self.session.auth = self.authentication

    def submit_request_callback(self, req):
        """ Service callback that submits a REST API request and returns the
            result

            Keyword arguments:
            req -- A request to send to the OSI PI server
        """
        # deserialize request object
        request = pickle.loads(literal_eval(req.request))
        # rospy.loginfo('Submitting request:\n\t'
        #               f'{pretty_print_request(request)}')
        try:
            response = self.session.send(request.prepare(), verify=False)
        except requests.exceptions.ConnectionError as exc:
            rospy.logerr('Failed to submit request to osi pi with: '
                         f'{str(exc)[:150]}'
                         '...' * (len(str(exc)) > 150))
            return JSONResponse(success=False,
                                response=requests.Response().text)

        success = False
        if (request.method, response.status_code) in (('GET', 200), ('POST', 202), ('POST', 207)):
            success = True
        elif response.status_code == 403:
            rospy.logwarn('OSI PI session expired')
            self.reset()
        elif response.status_code == 404:
            rospy.logerr(f'Invalid path:\n\t{pretty_print_request(response.json())}')
        elif response.status_code == None:
            rospy.logerr('Request to OSI PI failed. Possible connection failure')
        else:
            rospy.logerr('Invalid request')

        return JSONResponse(success=success, response=response.text)
