import sys
import os
import pickle
sys.path.append(
    '/home/kpv/catkin_ws/src/smelter-monitoring/src/slagsquare_pot_state_server/src'
    )
import numpy as np
import pandas as pd
from slagsquare_pot_state_server.crane_observer import CraneObserver
from slagsquare_pot_state_server.grid_model import GridModel

test_parameter_mapping = {
    'x': 'Positie brug H501',
    'x_discrete': 'H26.H501 Actuele rij',
    'y': 'Positie hoofdkat brug H501',
    'z': 'Hoogte hoofdhijs H501',
    'w': 'Gewicht hoofdhijs brug H501',
    }

test_files_dir = os.path.join(os.path.dirname(__file__),'test_files')
test_df = pickle.load(open(os.path.join(test_files_dir,'test_crane_data_may_2020.p'),"rb"))
test_df = test_df.resample('1S').mean().interpolate().dropna()

h_ir2rgb_path = os.path.join(test_files_dir,'test_homographies_ir_rgb.yml')
h_rgb2cad_path = os.path.join(test_files_dir,'test_homographies_rgb_cad.yml')

def test_constructor():
    crane_observer = CraneObserver(None,[None],None)
    assert crane_observer is not None,\
        "object should be instantiated"

def test_find_closest_pot():
    crane_observer = CraneObserver(None,[None],None)

    # test case 1: empty pot list
    response = crane_observer.find_closest_pot({},0.,0.,1.)
    assert response == [], "function should return empty list"

    # test case 2: single pot that is close
    pot_list = {
        'datetime': (0., None, 0., None),
    }
    response = crane_observer.find_closest_pot(pot_list,0.,0.,1.)
    assert response == ["datetime"], "function should return mock datetime key"

    # test case 3: single pot that is too far
    pot_list = {
        'datetime': (15., None, 15., None),
    }
    response = crane_observer.find_closest_pot(pot_list,0.,0.,1.)
    assert response == [], "function should return empty list"

    # test case 4: multiple pots with only one that is close
    pot_list = {
        'datetime1': (.2, None, .4, None),
        'datetime2': (15., None, 15., None),
    }
    response = crane_observer.find_closest_pot(pot_list,0.,0.,1.)
    assert response == ["datetime1"], "function should return mock datetime key"

    # test case 5: multiple pots that are both close
    pot_list = {
        'datetime1': (.2, None, .4, None),
        'datetime2': (.3, None, .9, None),
    }
    response = crane_observer.find_closest_pot(pot_list,0.,0.,1.)
    assert response == [], "function should return empty list"

    # test case 6: multiple pots that are both close, forced return
    pot_list = {
        'datetime1': (.2, None, .4, None),
        'datetime2': (.3, None, .9, None),
    }
    response = crane_observer.find_closest_pot(pot_list,0.,0.,1.,force_multiple=True)
    assert response == ['datetime1','datetime2'], "function should return all pots that are close"

def test_update_states():
    crane_observer = CraneObserver(None,[None],None)

    # test crane states
    crane_observer.update_crane_state_to(crane_observer.sp.MOVING)
    assert crane_observer.crane_state == crane_observer.sp.MOVING
    crane_observer.update_crane_state_to(crane_observer.sp.STOPPED)
    assert crane_observer.crane_state == crane_observer.sp.STOPPED

    # test load states
    crane_observer.update_load_state_to(crane_observer.sp.EMPTY)
    assert crane_observer.load_state == crane_observer.sp.EMPTY
    crane_observer.update_load_state_to(crane_observer.sp.FULL)
    assert crane_observer.load_state == crane_observer.sp.FULL
    crane_observer.update_load_state_to(crane_observer.sp.NONE)
    assert crane_observer.load_state == crane_observer.sp.NONE

    # test pot states
    crane_observer.update_pot_state_to(crane_observer.sp.COLD)
    assert crane_observer.pot_state == crane_observer.sp.COLD
    crane_observer.update_pot_state_to(crane_observer.sp.HOT)
    assert crane_observer.pot_state == crane_observer.sp.HOT
    crane_observer.update_pot_state_to(crane_observer.sp.UNKNOWN)
    assert crane_observer.pot_state == crane_observer.sp.UNKNOWN

def test_read():
    crane_observer = CraneObserver(None,[None],test_parameter_mapping)
    for row in test_df.iterrows():
        try:
            crane_observer.read(row)
        except:
            assert False, "update of crane parameters has failed"
        break

def build_test_gridmodel():
    """Returns a test grid_model"""
    sys.path.append(
        '/home/kpv/catkin_ws/src/smelter-monitoring/src/slagsquare_pot_state_server/test'
        )
    from test_GridModel import TestGridModel

    test_wrapper = TestGridModel()
    test_wrapper.setUp()

    return test_wrapper.grid

def test_process():
    """Tests the internal logic. Uses mock dataset."""
    max_test_iterations = 4000
    counter = 0
    print(f'processing {max_test_iterations} steps...')
    print('==========================================')
    crane_observer = CraneObserver(None,[build_test_gridmodel()],test_parameter_mapping)
    for row in test_df.iterrows():
        crane_observer.process(row)
        counter += 1
        if counter > max_test_iterations:
            break
    print('=====process test completed=====')

def test_all():
    test_constructor()
    test_read()
    test_process()
    test_find_closest_pot()
    test_update_states()

if __name__ == "__main__":
    test_all()
    print("Everything passed")
