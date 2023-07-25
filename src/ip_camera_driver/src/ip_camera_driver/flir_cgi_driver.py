import os
import time
import numpy as np
import cv2
import json
import re
import wget
import threading
import subprocess
import requests
from threading import Lock
from PIL import Image
from io import BytesIO
from flirimageextractor import FlirImageExtractor
from math import radians, degrees, ceil, pi

requests = requests.Session()
requests.trust_env = False

# set runtime warnings to raise instead of simple prints
np.seterr(all='warn')

REQUEST_ATTEMPTS = 5
IRNUCTable = {0: '-20oC-120oC', 1: '0oC-350oC', 2: '200oC-1200oC',
              3: '0oC-1200oC', 4: '-20oC-1200oC'}
IRAGC = {0: 'Auto', 1: 'Manual', 2: 'Linear', 3: 'Histogram'}
REQUIRED_THERMAL_INFO = ('RawThermalImage', 'Emissivity',
                         'AtmosphericTemperature',
                         'ReflectedApparentTemperature', 'IRWindowTemperature',
                         'IRWindowTransmission', 'RelativeHumidity', 'PlanckR1',
                         'PlanckB', 'PlanckF', 'PlanckO', 'PlanckR2')

class FlirCGIDriver:

    def __init__(self, ip='192.168.1.104', port='8090', timeout=40.0,
                 fff_storage_path=os.path.expanduser('~'),
                 pt_offsets={'pan': 0.0, 'tilt': 0.0}):
        self.ip = ip
        self.port = port
        self.url = str("http://" + self.ip + ":" + self.port + "/Nexus.cgi")
        self.timeout = timeout
        self.session_id = None
        self.session_lock = Lock()
        self.control_acquired = False
        self.remote_control_token = None
        self.temperature_range_mode = None
        self.pt_offsets = pt_offsets
        self.ping_status = False
        self.ping_timer = None
        self.request_lock = Lock()
        self.fff_storage_path = fff_storage_path

    def start_session(self, control=True, ping=True):
        with self.session_lock:
            self.session_id = self.request_session_id()
        if self.session_id is None:
            return False
        if control:
            self.remote_control_token = self.request_remote_control()
            self.control_acquired = (self.remote_control_token != None)
        if ping:
            self.ping_timer = threading.Timer(interval=0.2,
                                              function=self.ping_callback)
            self.ping_timer.start()
        return not control or self.control_acquired

    def restart_session(self):
        self.session_id = self.request_session_id()
        if self.session_id is None:
            return False
        if self.remote_control_token != None:
            self.remote_control_token = self.request_remote_control()
            self.control_acquired = (self.remote_control_token != None)
            return self.control_acquired
        else:
            return True

    def request_session_id(self):
        try:
            session_id = self.request_read_helper(None, 'SERVERWhoAmI', 'Id')
        except Exception as e: #requests.exceptions.ConnectionError as e:
            print('Camera failed to initialize with error: {}'.format(e))
            exit(0)
        return session_id

    def handle_request(self, request=None, retries=REQUEST_ATTEMPTS):
        if request is None:
            print('Invalid request provided [{}]'.format(request))
        if 'Set' in request['action'] and not self.control_acquired:
            print('Error: Request {} requires control'.format(request))
            return None
        for i in range(REQUEST_ATTEMPTS):
            #  print('request {} getting lock'.format(request))
            if request['action'] != 'SERVERPing':
                with self.request_lock:
                    response = requests.get(self.url, params=request).json()
            else:
                response = requests.get(self.url, params=request).json()
            #  print('request {} releasing lock'.format(request))
            if 'error' in response:
                if 'Return Code' in response['error'] and \
                        response['error']['Return Code'] == 21: # unauthorized
                    print('Restarting session due to expired client while handling request {}'.format(request))
                    if self.restart_session():
                        break
                    #  self.start_session(control=(self.control_acquired!=None),
                                       #  ping=True)
                else:
                    print('Request {} failed with: {}. {} attempts remaining.'.format(
                        request['action'], response, retries-i-1))
            elif response[request['action']]['Return Code'] == 11: # device busy
                #  print('Request {} failed with {}. {}# attempts remaining.'.format(
                    #  request['action'],response[request['action']]['Return String'],
                    #  REQUEST_ATTEMPTS-i))
                time.sleep(0.5)
            else:
                return response
        return None

    def handle_response(self, response, action, result=None,
                        checks=None):
        if response is None:
            return None
        if checks is None and response[action]['Return Code'] != 0:
            return None
        elif checks is not None:
            for key in checks:
                if response[action][key] != checks[key]:
                    return None
        if result is None:
            return response[action]
        elif isinstance(result, list) or isinstance(result, tuple):
            return [response[action][item] for item in result]
        return response[action][result]

    def check_result(self, result):
        return (result is not None) \
            and (not 'error' in result) \
            and (result['Return Code'] == 0)

    def request_read_helper(self, session_id=None, action=None, param=None,
                            checks=None):
        if action is None:
            raise ValueError('request_read_helper should not be called with action=None')
        request = {'action': action}
        if session_id is not None:
            request['session'] = session_id
        response = self.handle_request(request)
        value = self.handle_response(response, action, param, checks)
        return value

    def request_write_helper(self, session_id, action, param, value):
        request = {'session': session_id,
                   'action': action,
                   param: value}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        return self.check_result(result)

    def ping_callback(self):
        if not self.ping_cgi_server():
            self.ping_status = False
            print('FLIR Nexus Server ping failed')
        else:
            self.ping_status = True
            #  print('Pinging FLIR Nexus Server succeeded')
        self.ping_timer = threading.Timer(interval=0.2,
                                          function=self.ping_callback)
        self.ping_timer.start()

    def ping_cgi_server(self):
        with self.session_lock:
            if self.session_id is None:
                return False
            result = self.request_read_helper(self.session_id, 'SERVERPing')
        return self.check_result(result)

    def request_remote_control(self):
        request = {'session': self.session_id,
                   'action': 'SERVERRemoteControlRequest',
                   'Forced': 1}
        response = self.handle_request(request)
        token = self.handle_response(response, request['action'], 'Token',
                                     checks={'Return Code': 0, 'Accept': 0})
        return token

    def release_remote_control(self):
        return self.request_read_helper(self.session_id,
                                        'SERVERRemoteControlRelease',
                                        'RemoteID', self.remote_control_token)

    def request_server_reboot(self): # invalid function
        result = self.request_read_helper(self.session_id,
                                        'SERVERReboot',
                                        'Delay', 0)
        return self.check_result(result)

    def request_camera_reboot(self): # function unavailable
        result = self.request_read_helper(self.session_id,
                                        'IRReboot')
        return self.check_result(result)

    def request_snapshot_storage(self, session_id, format_id=0):
        request = {'session': self.session_id,
                   'action': 'IRTHGSnapshotStore',
                   'Format': format_id}
        response = self.handle_request(request)
        snapshot_id = self.handle_response(response, request['action'], 'Id')
        return snapshot_id

    def request_snapshot_url(self, snapshot_id):
        request = {'session': self.session_id,
                   'action': 'IRTHGSnapshotUrlGet',
                   'Id': snapshot_id}
        response = self.handle_request(request)
        snapshot_url = self.handle_response(response, request['action'], 'Url')
        if snapshot_url is None:
            return None
        return str("http://" + self.ip + ":" + self.port + snapshot_url)

    def request_snapshot_state(self, snapshot_url):
        ''' Test if requested snapshot is available for download '''
        Id = re.search('capture/(.+?)_IR0', snapshot_url).group(1)
        request = {'session': self.session_id,
                   'action': 'IRTHGSnapshotStateGet',
                   'Id': Id}
        response = self.handle_request(request)
        state = self.handle_response(response, request['action'], 'State')
        return state

    def request_temperature_range_mode(self, index=None):
        if self.temperature_range_mode == index:
            return True
        request = {'session': self.session_id,
                   'action': 'IRNUCTableSet',
                   'NUC_Index': index}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        if self.check_result(result):
            self.temperature_range_mode = index
            return True
        else:
            return False

    def request_agc(self, index=None):
        if not self.control_acquired:
            print('Cannot change AGC mode without control')
            return False
        request = {'session': self.session_id, 'action': 'IRAGCSet',
                   'Agc': index}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        return self.check_result(result)

    def request_pan_tilt_initialization(self):
        request = {'session': self.session_id,
                   'action': 'PTInitialize'}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        return self.check_result(result)

    def request_home_position(self):
        request = {'session': self.session_id,
                   'action': 'PTSendHome'}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        return self.check_result(result)

    def request_pan_tilt_write(self, pan=0.0, tilt=0.0):
        request = {'session': self.session_id,
                   'action': 'PTAzimuthElevationSet',
                   'Azimuth': degrees(pan + self.pt_offsets['pan']),
                   'Elevation': degrees(tilt + self.pt_offsets['tilt'])}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        return self.check_result(result)

    def request_pan_tilt_read(self):
        request = {'session': self.session_id,
                   'action': 'PTAzimuthElevationGet'}
        response = self.handle_request(request)
        ret = self.handle_response(response, request['action'],
                                          ['Azimuth', 'Elevation'])
        if ret is None:
            return ret
        azimuth, elevation = ret[0], ret[1]
        pan, tilt = radians(azimuth), radians(elevation)
        pan, tilt = self.normalize_radians(pan), self.normalize_radians(tilt)
        pan -= self.pt_offsets['pan']
        tilt -= self.pt_offsets['tilt']
        pan, tilt = round(pan, 3), round(tilt, 3)
        return pan, tilt

    def request_ir_zoom_read(self):
        return self.request_read_helper(
            self.session_id, 'IRZoomPercentageGet', 'Zoom')

    def request_visual_zoom_read(self):
        return self.request_read_helper(
            self.session_id, 'DLTVZoomPercentageGet', 'Zoom')

    def request_ir_zoom_write(self, zoom):
        request = {'session': self.session_id, 'action': 'IRZoomPercentageSet',
                   'Zoom': zoom}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        return self.check_result(result)

    def request_vis_zoom_write(self, zoom):
        request = {'session': self.session_id, 'action': 'IRZoomPercentageSet',
                   'Zoom': zoom}
        response = self.handle_request(request)
        result = self.handle_response(response, request['action'])
        return self.check_result(result)

    def request_zoom_slave_read(self):
        zoom_slave_states = {}
        for action in ['IRZoomSlaveGet', 'DLTVZoomSlaveGet']:
            zoom_slave_states[action[:-3]] = self.request_read_helper(
                self.session_id, action, 'Slave_Zoom')
        return zoom_slave_states

    def request_zoom_slave_write(self, ir=False, vis=False):
        for cam, slave in zip(['IR', 'DLTV'], [ir, vis]):
            if not self.request_write_helper(self.session_id,
                    cam+'ZoomSlaveSet', 'Slave_Zoom', int(slave)):
                return False
        if not self.write_ptz(ir_zoom=0, vis_zoom=8.2):
            return False
        return True

    def read_ptz(self):
        pt_ret = self.request_pan_tilt_read()
        ir_zoom = self.request_ir_zoom_read()
        vis_zoom = self.request_visual_zoom_read()
        if pt_ret is None or ir_zoom is None or vis_zoom is None:
            return None
        pan, tilt = pt_ret
        return {
            'pan': pan,
            'tilt': tilt,
            'ir_zoom': ir_zoom,
            'vis_zoom': vis_zoom
        }

    def write_ptz(self, pan=None, tilt=None, ir_zoom=None, vis_zoom=None):
        ret = True
        if None not in [pan, tilt]:
            ret *= self.request_pan_tilt_write(pan, tilt)
        if ir_zoom is not None:
            ret *= self.request_ir_zoom_write(ir_zoom)
        if vis_zoom is not None:
            ret *= self.request_vis_zoom_write(vis_zoom)
        return ret

    def read_raw_snapshot(self, format_id=0):
        snapshot_id = self.request_snapshot_storage(self.session_id,
                                                    format_id=format_id)
        snapshot_url = self.request_snapshot_url(snapshot_id)
        if snapshot_url is None or not self.wait_for_snapshot(snapshot_url):
            return None, None
        metadata = self.wait_for_metadata(snapshot_url)
        if metadata is None:
            return None, None
        metadata['timestamp'] = snapshot_id
        snapshot = self.fetch_snapshot(snapshot_url)
        if snapshot is not None and self.fff_storage_path is not None:
            self.save_fff_snapshot(snapshot_url)
        return metadata, snapshot

    def save_fff_snapshot(self, url):
        print("Downloading snapshot from url: " + str(url))
        try:
            wget.download(url, out=self.fff_storage_path)
        except Exception as e:
            print('Failed to download fff snapshot from url={} with error: '
                  '{}'.format(url, e))
            return False
        return True

    def wait_for_snapshot(self, snapshot_url):
        ts = time.time()
        while time.time()-ts < self.timeout:
            snapshot_state = self.request_snapshot_state(snapshot_url)
            ''' When selectin format_id==0 (FFF) we get snapshot_state==4
            although the snapshot is available. It's a bug in the firmware
            confirmed by FLIR.
            '''
            if snapshot_state in [0, 4]:
                print('Radiometric snapshot is available: %s'%snapshot_url)
                break
            elif snapshot_state == 0:
                time.sleep(1)
            else:
                if snapshot_state == 3:
                    print('Radiometric snapshot not found')
                if snapshot_state == 4:
                    print('Radiometric snapshot Error')
                return False
        else:
            print('Reading radiometric snapshot timed out')
            return False
        return True

    def wait_for_metadata(self, snapshot_url):
        metadata = None
        ts = time.time()
        while time.time()-ts < self.timeout:
            try:
                metadata = self.fetch_snapshot_metadata(snapshot_url)
            except:
                metadata = {}
            # metadata must contains all required thermal info
            if not (set(REQUIRED_THERMAL_INFO) - set(metadata.keys())):
                return metadata
            else:
                #  print('Radiometric snapshot metadata not available yet')
                time.sleep(1)
        print('Reading radiometric snapshot metadata failed. Latest metadata: '
              '{}'.format(metadata))
        return None

    def read_snapshot(self, format_id=0, range_mode=0):
        if range_mode in [0,1,2]:
            return self.read_thermal_snapshot(format_id, range_mode)
        elif range_mode == 3: # combine 1 and 2 to get 0-1200oC range
            return self.read_combined_thermal_snapshot(format_id, zero=False)
        elif range_mode == 4: # combine 1,2,3 to get -20-1200oC range
            return self.read_combined_thermal_snapshot(format_id, zero=True)
        return None, None

    def read_thermal_snapshot(self, format_id=0, range_mode=0):
        if not self.request_temperature_range_mode(index=range_mode):
            print('Failed to set temperature range mode {}'.format(
                IRNUCTable[range_mode]))
            return None, None
        metadata, snapshot = self.read_raw_snapshot(format_id)
        if metadata is None or snapshot is None:
            print('Failed to read snapshot with temperature range {}'.format(
                IRNUCTable[range_mode]))
            return None, None
        thermal_snapshot = self.raw_to_temperatures(snapshot, metadata)
        return metadata, thermal_snapshot

    def read_combined_thermal_snapshot(self, format_id=0, zero=False):
        # read 0-350oC snapshot
        metadata1, snapshot1 = self.read_thermal_snapshot(format_id, 1)
        # read 200-1200oC snapshot
        metadata2, snapshot2 = self.read_thermal_snapshot(format_id, 2)
        if None in [metadata1, metadata2] or \
                snapshot1 is None or snapshot2 is None:
            print('Failed to read snapshots to combine. '
                  'm1: {}, s1: {}, m2: {}, s2: {}'.format(
                  type(metadata1), type(snapshot1),
                      type(metadata2), type(snapshot2)))
            return (None, None), None
        # merge snapshots for 0-1200oC range
        snapshot2[snapshot2 - 350.0 <= 0.0] = -100.0
        if zero: # read -20-120oC snapshot
            metadata0, snapshot0 = self.read_thermal_snapshot(format_id, 0)
            snapshot1[snapshot1 - 120.0 <= 0.0] = -100.0
            #  merge snapshots for -20-1200oC range
            snapshot = np.max((snapshot0, snapshot1, snapshot2), axis=0)
            return (metadata0, metadata1, metadata2), snapshot
        snapshot = np.max((snapshot1, snapshot2), axis=0)
        return (metadata1, metadata2), snapshot

    def mock_read_raw_snapshot(self, url):
        metadata = self.fetch_snapshot_metadata(url)
        snapshot = self.fetch_snapshot(url)
        return metadata, snapshot

    def mock_read_snapshot(self, url):
        metadata, snapshot = self.mock_read_raw_snapshot(url)
        thermal_snapshot = self.raw_to_temperatures(snapshot, metadata)
        return metadata, thermal_snapshot

    def fetch_snapshot_metadata(self, snapshot_url):
        try:
            ps = subprocess.Popen(['curl', '-L', '-s', snapshot_url],
                                  stdout=subprocess.PIPE)
            raw_metadata = subprocess.check_output(['exiftool', '-j', '-'],
                                                   stdin=ps.stdout)
        except Exception as e: # TODO handle specific exception
            #  print('Failed to read radiometric snapshot metadata: %s'%e)
            return {}
        metadata = json.loads(raw_metadata)[0]
        return metadata

    def fetch_snapshot(self, snapshot_url):
        try:
            ps = subprocess.Popen(['curl', '-L', '-s', snapshot_url],
                                  stdout=subprocess.PIPE)
            raw = subprocess.check_output(
                ['exiftool', '-b', '-RawThermalImage', '-'], stdin=ps.stdout)
        except Exception as e: # TODO handle specific exceptions
            print('Failed to fetch radiometric snapshot with exception: %s'%e)
            return None
        try:
            dataBytesIO = BytesIO(raw)
            dataBytesIO.seek(0)
            pil_img = Image.open(dataBytesIO)
            cv_img = np.array(pil_img)
        except Exception as e:
            print('Failed to process radiometric snapshot with exception: %s'%e)
            return None
        return cv_img

    def raw_to_temperatures(self, snapshot, metadata):
        if snapshot is None or metadata is None:
            return snapshot
        subject_distance = 5.0
        if 'SubjectDistance' in metadata:
            try:
                subject_distance = FlirImageExtractor.extract_float(
                    metadata['SubjectDistance'])
            except:
                pass
        if metadata['RawThermalImageType'].upper().strip() == 'TIFF':
            fix_endian = False
        if fix_endian:
            snapshot = np.vectorize(lambda x: (x >> 8) + ((x & 0x00ff) << 8))(snapshot)
        raw2tempfunc = np.vectorize(lambda x: FlirImageExtractor.raw2temp(x,
            E=metadata['Emissivity'], OD=subject_distance,
            RTemp=FlirImageExtractor.extract_float(metadata['ReflectedApparentTemperature']),
            ATemp=FlirImageExtractor.extract_float(metadata['AtmosphericTemperature']),
            IRWTemp=FlirImageExtractor.extract_float(metadata['IRWindowTemperature']),
            IRT=metadata['IRWindowTransmission'],
            RH=FlirImageExtractor.extract_float(metadata['RelativeHumidity']),
            PR1=metadata['PlanckR1'], PB=metadata['PlanckB'],
            PF=metadata['PlanckF'],
            PO=metadata['PlanckO'], PR2=metadata['PlanckR2']))
        try:
            thermal_snapshot = raw2tempfunc(snapshot)
        except RuntimeWarning as e:
            print('Converting pixel values to temperatures failed with warning: {}\n'
                  'metadata:\n{}'.format(e, metadata))
        except ValueError as e:
            print('Converting pixel values to temperatures failed with error: {}\n'
                  'metadata:\n{}'.format(e, metadata))
            return None
        clip_lower = FlirImageExtractor.extract_float(metadata['CameraTemperatureMinClip'])
        clip_upper = FlirImageExtractor.extract_float(metadata['CameraTemperatureMaxClip'])
        thermal_snapshot = np.nan_to_num(thermal_snapshot, clip_lower)
        thermal_snapshot = np.clip(thermal_snapshot, clip_lower, clip_upper)
        return thermal_snapshot

    def normalize_degrees(self, angle):
        return angle + ceil((-angle - pi) / 2 / pi) * 2 * pi

    def normalize_radians(self, angle):
        return angle + ceil((-angle - pi) / 2 / pi) * 2 * pi
