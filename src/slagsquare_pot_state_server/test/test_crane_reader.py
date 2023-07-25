import sys
import os
import pickle
sys.path.append(os.path.dirname(sys.path[0]))
from slagsquare_pot_state_server.crane_reader import CraneReader

test_config = {
        'PIWEBAPI_URL': 'https://vpc-as-0075.nucleus.atom.ads/piwebapi',
        'AF_SERVER_NAME': 'HOB',
        'USER_NAME': 'x000229@umicore.com',
        'USER_PASSWORD': '6Q%;H4xyi>n=,%l',
        'AUTH_TYPE': 'basic'
    }

test_web_ids = {
    'Positie brug H501': 'F1DPYUB_fvptv0iuk9DWiEl8XwolsAAAVlBDLUFTLTAwNzVcSDI2LkdJNTAxLjAxLlBW',
    'Positie hoofdkat brug H501': 'F1DPYUB_fvptv0iuk9DWiEl8Xwp1sAAAVlBDLUFTLTAwNzVcSDI2LkdJQ0E1MDEuMjAuUFY',
    'H26.H501 Actuele rij': 'F1DPYUB_fvptv0iuk9DWiEl8Xww18AAAVlBDLUFTLTAwNzVcSDI2Lkg1MDEgQUNUVUVMRSBSSUo',
    'Hoogte hoofdhijs H501': 'F1DPYUB_fvptv0iuk9DWiEl8Xw-1sAAAVlBDLUFTLTAwNzVcSDI2LkdTNTAxLjIyLlBW',
    'Gewicht hoofdhijs brug H501': 'F1DPYUB_fvptv0iuk9DWiEl8XwQGkAAAVlBDLUFTLTAwNzVcSDI2LldJU0E1MDEuMjAuMi5QVg',
}

def test_constructor():
    crane_reader = CraneReader(config=test_config,web_ids=test_web_ids)
    assert crane_reader is not None

def test_get_current_minute():
    last_minute = CraneReader.get_current_minute()
    print(last_minute)
    assert last_minute.second == 0, 'datetime should be rounded down to the minute'

def test_read_from_stream_multiple_recorded():
    crane_reader = CraneReader(config=test_config,web_ids=test_web_ids)
    try:
        response = crane_reader.read_from_stream_multiple_recorded(crane_reader.web_ids)
    except:
        print('[WARNING]: read_from_stream_multiple_recorded() failed.')
        print('           This test only makes sense when connected to the osi pi server.')

if __name__ == "__main__":
    test_constructor()
    test_get_current_minute()
    test_read_from_stream_multiple_recorded()
    print("Everything passed")
