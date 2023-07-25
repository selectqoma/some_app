#!/usr/bin/env python3

import sys
import pytest
import rospy
import pickle
from ast import literal_eval
from smelter_osi_pi.osi_pi_utils import *

from smelter_srvs.srv import JSON, JSONRequest, JSONResponse


PIWEBAPI_URL = 'https://devdata.osisoft.com/piwebapi'
AF_SERVER_NAME = 'PISRV1'
PI_SERVER_NAME = 'PIServerName'
USER_NAME = 'webapiuser'
USER_PASSWORD = '!try3.14webapi!'
AUTH_TYPE = 'basic'

OSI_AF_ATTRIBUTE_TAG = 'RawData'
OSI_AF_DATABASE = 'Forecasts'
OSI_AF_ELEMENT = 'CDF144'

TAG_WEB_ID = 'F1AbEIRAQC7zjPUOfBqai218IAwifUOTEAv5xGAzgANOhDHzg4VRBlNqet1wMQswtjrtN2AUElTUlYxXEZPUkVDQVNUU1xDREYxNDR8UkFXREFUQQ'

@pytest.fixture
def node():
    rospy.init_node('osi_pi_proxy_test', anonymous=True)

def test_tag_id_reading_proxy(node):

    # wait until service is available / node is launched
    rospy.wait_for_service('/osi_pi/submit_request', timeout=5)

    # initialize a service request handler
    submit_request = rospy.ServiceProxy('/osi_pi/submit_request', JSON)

    request = construct_get_request(PIWEBAPI_URL, AF_SERVER_NAME, USER_NAME,
                              USER_PASSWORD, AUTH_TYPE, OSI_AF_DATABASE,
                              [OSI_AF_ELEMENT], OSI_AF_ATTRIBUTE_TAG)
    serialized_request = serialize_request(request)
    request_msg = JSONRequest(request=serialized_request)

    # send request
    response = submit_request(request_msg)

    info = json.loads(response.response)

    assert response.success
    assert 'WebId' in info
    assert isinstance(info['WebId'], str)

    web_id = info['WebId']
    assert web_id == TAG_WEB_ID


def test_tag_id_batch_reading_through_proxy(node):

    # wait until service is available / node is launched
    rospy.wait_for_service('/osi_pi/submit_request', timeout=5)

    # initialize a service request handler
    submit_request = rospy.ServiceProxy('/osi_pi/submit_request', JSON)

    osi_af_attributes_batch = {}
    osi_af_attributes_batch['1'] = {
        'elements_seq': [OSI_AF_ELEMENT],
        'attributes': {
            'raw_data': OSI_AF_ATTRIBUTE_TAG
        }
    }
    osi_af_attributes_batch['2'] = {
        'elements_seq': [OSI_AF_ELEMENT],
        'attributes': {
            'raw_data': OSI_AF_ATTRIBUTE_TAG
        }
    }
    #  raise ValueError(osi_af_attributes_batch)

    request = construct_batch_read_attribute_info(
        PIWEBAPI_URL, AF_SERVER_NAME, USER_NAME, USER_PASSWORD, AUTH_TYPE,
        OSI_AF_DATABASE, osi_af_attributes_batch)

    serialized_request = serialize_request(request)
    request_msg = JSONRequest(request=serialized_request)

    # send request
    response = submit_request(request_msg)

    info = json.loads(response.response)

    assert response.success

    for key in info:
        assert 'WebId' in info[key]['Content']
        assert isinstance(info[key]['Content']['WebId'], str)
        assert info[key]['Content']['WebId'] == TAG_WEB_ID


def test_tag_value_reading_through_proxy(node):
    assert TAG_WEB_ID is not None

    # wait until service is available / node is launched
    rospy.wait_for_service('/osi_pi/submit_request')

    # initialize a service request handler
    submit_request = rospy.ServiceProxy('/osi_pi/submit_request', JSON)

    # construct request
    request = construct_get_request_with_web_id(PIWEBAPI_URL, TAG_WEB_ID,
                                                USER_NAME, USER_PASSWORD,
                                                AUTH_TYPE)
    # serialize request
    serialized_request = str(pickle.dumps(request))

    # perform tests
    request_msg = JSONRequest(request=serialized_request)

    # send request
    response = submit_request(request_msg)

    info = json.loads(response.response)

    assert response.success
    assert 'Timestamp' in info
    assert 'Value' in info
    assert isinstance(info['Value'], float)


"""
def test_tag_value_writing_through_proxy(node):
    assert True
"""
