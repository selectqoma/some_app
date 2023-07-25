""" osi_pi_utils - This module includes functions used to interact (read/write)
    with an OSI Pi Server

    Reference:
    https://github.com/osisoft/OSI-Samples-PI-System/tree/master/piwebapi_samples/Python
"""

import json
import pickle
import requests
# import requests_kerberos
from requests.auth import HTTPBasicAuth
# from requests_kerberos import HTTPKerberosAuth
from pprint import pprint

# disable warnings about insecure / unverified https request
import urllib3


def call_headers(include_content_type):
    """ Create API call headers

        Keyword arguments:
        includeContentType -- flag that determines whether or not the
        content-type header is included
    """
    if include_content_type is True:
        header = {
            'content-type': 'application/json',
            'X-Requested-With': 'XmlHttpRequest'
        }
    else:
        header = {
            'X-Requested-With': 'XmlHttpRequest'
        }

    return header


def call_security_method(security_method, user_name, user_password):
    """ Create API call security method

        Keyword arguments:
        security_method --  security method to use basic or kerberos
        user_name -- The user's credentials name
        user_password -- The user's credentials password
    """
    if security_method.lower() == 'basic':
        security_auth = HTTPBasicAuth(user_name, user_password)
    else:
        raise ValueError('Only basic authentication is available')
        #  security_auth = HTTPKerberosAuth(mutual_authentication='REQUIRED',
                                         #  sanitize_mutual_error_response=False)

    return security_auth


def read_attribute_info(piwebapi_url, asset_server, user_name,
                        user_password, piwebapi_security_method,
                        osi_af_database, osi_af_element_path,
                        osi_af_attribute_tag):
    """ Read and return the info of the given attribute

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_element_path -- The sequence of elements to access
        osi_af_attribute_tag -- The name of the attribute to read
    """
    request = construct_get_request(piwebapi_url, asset_server,
                                    user_name, user_password,
                                    piwebapi_security_method,
                                    osi_af_database, osi_af_element_path,
                                    osi_af_attribute_tag)

    try:
        response = requests.Session().send(request.prepare(), verify=False,
                                           timeout=1)
    except requests.exceptions.ConnectionError as e:
        #  print(f'Failed to read from osi pi with error: {e}')
        return None

    # return response if valid else return None
    if response.status_code == 200:
        return json.loads(response.text)

    return None


def construct_batch_read_attribute_info(
        piwebapi_url, asset_server, user_name, user_password,
        piwebapi_security_method, osi_af_database, osi_af_attributes_batch):
    """ Create a request for batch reading tag web ids

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_attributes_batch -- Contains the elements sequence and the attributes
        of which the attributes to read
    """
    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    #  Create the header
    header = call_headers(False)

    # create batch request dictionary
    batch_request = {}

    # iterate through each item of the batch
    for key in osi_af_attributes_batch:
        elements_seq = osi_af_attributes_batch[key]['elements_seq']

        #  iterate through all attributes for each item
        for attr in osi_af_attributes_batch[key]['attributes']:
            # construct the attribute url
            url = '{}/attributes?path=\\\\{}\\{}\\{}|{}'.format(
                piwebapi_url, asset_server, osi_af_database,
                '\\'.join(elements_seq),
                osi_af_attributes_batch[key]['attributes'][attr])

            batch_request[str((key, attr))] = {
                'Method': 'GET',
                'Resource': url,
                'Content': '{}'
            }

    request = requests.Request('POST',
        piwebapi_url + '/batch', auth=security_method, headers=header,
        json=batch_request)

    return request


def read_batch_attribute_info(piwebapi_url, asset_server, user_name,
                              user_password, piwebapi_security_method,
                              osi_af_database, osi_af_attributes_batch):
    """ Read and return the info of the given attributes in the batch

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_attributes_batch -- Contains the elements sequence and the attributes
        of which the attributes to read
    """
    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    #  Create the header
    header = call_headers(False)

    # construct batch request
    request = construct_batch_read_attribute_info(
        piwebapi_url, asset_server, user_name, user_password,
        piwebapi_security_method, osi_af_database, osi_af_attributes_batch)

    try:
        response = requests.Session().send(request.prepare(), verify=False)
    except requests.exceptions.ConnectionError as e:
        print(f'Failed to batch read tag web ids from osi pi with error: {e}')
        return None

    if response.status_code == 207:
        print(f'Batch read status: {response.status_code}')
    else:
        print(response.status_code, response.reason)
        return None

    return response

def construct_get_request(piwebapi_url, asset_server, user_name,
                          user_password, piwebapi_security_method,
                          osi_af_database, osi_af_element_path,
                          osi_af_attribute_tag):
    """ Construct a request based on the given info

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_element_path -- The sequence of elements to access
        osi_af_attribute_tag -- The name of the attribute to read
    """

    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    #  Get the sample tag
    url = '{}/attributes?path=\\\\{}\\{}\\{}|{}'.format(
        piwebapi_url, asset_server, osi_af_database,
        '\\'.join(osi_af_element_path), osi_af_attribute_tag)

    request = requests.Request('GET', url, auth=security_method)

    return request


def construct_get_request_with_web_id(piwebapi_url, tag_web_id, user_name,
                                      user_password, piwebapi_security_method):
    """ Construct a request based on the given tag web id

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        tag_web_id -- The web id of the tag to read
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
    """

    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    #  Get the sample tag
    url = f'{piwebapi_url}/streams/{tag_web_id}/value'

    request = requests.Request('GET', url, auth=security_method)

    return request


def get_tag_read_url(piwebapi_url, asset_server, user_name,
                     user_password, piwebapi_security_method,
                     osi_af_database, osi_af_element_path,
                     osi_af_attribute_tag):
    """ Read and return the url for reading the value of an attribute

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_element_path -- The sequence of elements to access
        osi_af_attribute_tag -- The name of the attribute to read
    """
    info = read_attribute_info(piwebapi_url, asset_server, user_name,
                               user_password, piwebapi_security_method,
                               osi_af_database, osi_af_element_path,
                               osi_af_attribute_tag)
    if info is None:
        return info

    return info['WebId']


def get_tag_write_url(piwebapi_url, asset_server, user_name,
                      user_password, piwebapi_security_method,
                      osi_af_database, osi_af_element_path,
                      osi_af_attribute_tag):
    """ Read and return the url for writing values to the given attribute

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_element_path -- The sequence of elements to access
        osi_af_attribute_tag -- The name of the attribute to write to
    """
    info = read_attribute_info(piwebapi_url, asset_server, user_name,
                               user_password, piwebapi_security_method,
                               osi_af_database, osi_af_element_path,
                               osi_af_attribute_tag)
    if info is None:
        return info

    return info['Links']['Value']


def read_attribute_snapshot(piwebapi_url, asset_server, user_name,
                            user_password, piwebapi_security_method,
                            osi_af_database, osi_af_element_path,
                            osi_af_attribute_tag):
    """ Read a single value

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_element_path -- The sequence of elements to access
        osi_af_attribute_tag -- The name of the attribute to read
    """

    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    #  Get the sample tag
    request_url = '{}/attributes?path=\\\\{}\\{}\\{}|{}'.format(
        piwebapi_url, asset_server, osi_af_database,
        '\\'.join(osi_af_element_path), osi_af_attribute_tag)
    try:
        response = requests.get(request_url, auth=security_method, verify=False,
                                timeout=1)
    except requests.exceptions.ConnectionError as e:
        print(f'Failed to read from osi pi with error: {e}')
        return False, None

    #  Only continue if the first request was successful
    result = None
    if response.status_code == 200:
        #  Deserialize the JSON Response
        data = response.json()

        #  Read the single stream value
        try:
            response = requests.get(piwebapi_url + '/streams/' + data['WebId'] + \
                                    '/value', auth=security_method, verify=False,
                                    timeout=1)
        except requests.exceptions.ConnectionError as e:
            print(f'Failed to read from osi pi with error: {e}')
            return False, None

        if response.status_code != 200:
            result = json.dumps(response.json())
        #  else:
            #  print(response.status_code, response.reason, response.text)
    #else:
        #print(response.status_code, response.reason, response.text)

    return response.status_code == 200, result


def write_single_value(piwebapi_url, asset_server, user_name, user_password,
                       piwebapi_security_method, osi_af_database,
                       osi_af_element_path, osi_af_attribute_tag,
                       data_value):
    """ Write a single value to the sampleTag

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        asset_server -- Name of the Asset Server
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        osi_af_database -- The name of the database to query in OSI PI
        osi_af_element_path -- The sequence of elements to access
        osi_af_attribute_tag -- The name of the attribute to write
        data_value -- The value to write to the tag
    """

    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    # Get the sample tag
    request_url = '{}/attributes?path=\\\\{}\\{}\\{}|{}'.format(
        piwebapi_url, asset_server, osi_af_database,
        "\\".join(osi_af_element_path), osi_af_attribute_tag)
    #request_url = '{}/attributes?path=\\\\{}\\{}'.format(
    #    "https://vpc-as-0075.nucleus.atom.ads/piwebapi", "VPC-AS-0075",
    #    osi_af_attribute_tag)

    #print(request_url)
    try:
        response = requests.get(request_url, auth=security_method, verify=False,
                                timeout=1)
    except requests.exceptions.ConnectionError as e:
        print(f'Failed to read from osi pi with error: {e}')
        return False

    #  Only continue if the first request was successful
    if response.status_code == 200:
        #  Deserialize the JSON Response
        data = json.loads(response.text)

        #  Create the data for this call
        request_body = {
            'Value': data_value,
        }

        #  Create the header
        header = call_headers(True)

        #  Write the single value to the tag
        try:
            response = requests.post(data['Links']['Value'], auth=security_method,
                                     verify=False, json=request_body,
                                     headers=header, timeout=1)
        except requests.exceptions.ConnectionError as e:
            print(f'Failed to write to osi pi with error: {e}')
            return False
        #  if response.status_code == 202:
            #  print(response.status_code, response.reason, response.text)
    #  else:
        #  print(response.status_code, response.reason, response.text)

    return response.status_code == 202


def construct_batch_request(piwebapi_url, user_name, user_password,
                            piwebapi_security_method, batch_request):
    """ Write a batch request

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        batch_request -- Contains the requests to write
    """
    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    #  Create the header
    header = call_headers(True)

    # construct request
    request = requests.Request('POST',
        piwebapi_url + '/batch', auth=security_method,
        json=batch_request, headers=header)

    return request


def write_batch_request(piwebapi_url, user_name, user_password,
                        piwebapi_security_method, batch_request):
    """ Write a batch request

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        batch_request -- Contains the requests to write
    """
    request = construct_batch_request(
        piwebapi_url, user_name, user_password, piwebapi_security_method,
        batch_request)

    # post request
    try:
        response = requests.Session().send(request.prepare(), verify=False)
    except requests.exceptions.ConnectionError as e:
        #  print(f'Failed to write to osi pi with error: {e}')
        return False

    #  print('Batch Status: ' + str(response.status_code))

    return response.status_code == 207


def construct_streamset_read_request(piwebapi_url, user_name, user_password,
                                     piwebapi_security_method, web_ids,
                                     start_time, end_time):
    """ Construct a streamset read request

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        web_ids -- A list of web ids to read
        start_time -- The start time of the streamsets
        end_time -- The end time of the streamsets
    """
    #  create security method - basic or kerberos
    security_method = call_security_method(
        piwebapi_security_method, user_name, user_password)

    #  Create the header
    headers = call_headers(True)

    # create the payload for the streamsets
    payload = {
        'webId': web_ids,
        'startTime': str(start_time) + ' GMT',
        'endTime': str(end_time) + ' GMT'
        }

    # construct the url to send the request to
    base_url = piwebapi_url + '/streamsets/recorded'

    request = requests.Request('GET',
        base_url, params=payload, auth=security_method, headers=headers)

    return request


def send_streamset_read_request(piwebapi_url, user_name, user_password,
                                piwebapi_security_method, web_ids,
                                start_time, end_time):
    """ Returns a collection of recorded stream values over a given time window.

        This functions should be preferred over direct value reading as it puts
        less stress on the network.

        Keyword arguments:
        piwebapi_url -- the URL of the PI Web API
        user_name -- The user's credentials name
        user_password -- The user's credentials password
        piwebapi_security_method -- Security method basic or kerberos
        web_ids -- The web ids of the tags to read the streamsets of
        start_time -- The start time of the streamsets
        end_time -- The end time of the streamsets

        Returns:
        response (dict): request response in json format
    """
    request = construct_streamset_read_request(
        piwebapi_url, user_name, user_password, piwebapi_security_method,
        web_ids, start_time, end_time)

    # send the request and get the response
    try:
        response = requests.Session().send(request.prepare(), verify=False)
    except requests.exceptions.ConnectionError as e:
        print(f'Failed to read streamset of multiple web ids with: {e}')
        return None

    return response.json()


def serialize_request(request):
    """ Converts a request object into a pickle and then serializes it

        Keyword arguments:
        request -- A REST api request object
    """
    return str(pickle.dumps(request))


def pretty_print_request(req):
    """ Pretty prints a REST API request

        Keyword arguments:
        req -- The request to pretty print
    """
    if req.method == 'GET':
        pretty_print_get(req)
    else:
        pretty_print_post(req)


def pretty_print_get(req):
    """ Pretty prints a REST API GET request

        Keyword arguments:
        req -- The GET request to pretty print
    """
    print('{}\n{}\r\nusername: {}\r\npassword: {}\r\n{}\r\n\r'.format(
        '-----------START-----------',
        req.method + ' ' + req.url,
        req.auth.username,
        req.auth.password,
        '\r\n'.join('{}: {}'.format(k, v) for k, v in req.headers.items())
    ))


def pretty_print_post(req):
    """ Pretty prints a REST API POST request

        Keyword arguments:
        req -- The POST request to pretty print
    """
    print('{}\n{}\r\n{}\r\n\r\n{}'.format(
        '-----------START-----------',
        req.method + ' ' + req.url,
        '\r\n'.join('{}: {}'.format(k, v) for k, v in req.headers.items()),
        req.json,
    ))


def pretty_print_response(response):
    pprint(response.json())
