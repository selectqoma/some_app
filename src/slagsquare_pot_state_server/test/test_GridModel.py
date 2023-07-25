import sys
import time
import os
sys.path.append(
    '/home/kpv/catkin_ws/src/smelter-monitoring/src/slagsquare_pot_state_server/src'
    )
import unittest

from slagsquare_pot_state_server.grid_model import *
from slagsquare_pot_state_server.grid_model_utils import *

## A sample python unit test
class TestGridModel(unittest.TestCase):
    
    def setUp(self):
        test_files_dir = os.path.join(os.path.dirname(__file__),'test_files')

        h_ir2rgb_path = os.path.join(test_files_dir,'test_homographies_ir_rgb.yml')
        h_rgb2cad_path = os.path.join(test_files_dir,'test_homographies_rgb_cad.yml')

        self.grid = GridModel(h_ir2rgb_path, h_rgb2cad_path, rows=8, cols=20)
        self.grid.setRgbParameters(480, 704)  # leave hardcoded for now
        self.grid.setCadParameters(os.path.join(test_files_dir,'test_ground_plan.png'))
        self.grid.setCraneGridMappingParameters(
                {
                    'x_start':  41.5,
                    'x_end':    83.5,
                    'y_start':  1.3,
                    'y_end':    17.5,
                }
            )
        self.grid.setOccupancyGrid()  # set rows and columns
        self.grid.useColumnOffsets = True
        self.grid.validationMargin = 0.7
        self.grid.setPartialFunctions()

    def tearDown(self):
        self.grid = None

    def test_calculate_column_offsets(self):
        self.grid.patrolMaskCenters = np.array([[]])
        test_offsets = self.grid.calculateColumnOffsets()
        self.assertEqual(self.grid.columnOffsets.all(), test_offsets.all(), "column offsets should remain the same")

    def test_validation_functions(self):
        print('Calculating validation map via the slow method. This can take around 30s.')
        t_start = time.time()
        img_slow = getValidationImage(self.grid.cadImage,self.grid.validationFunction)
        t_slow = time.time()-t_start
        print(f'Slow method took {t_slow:.2f}s')

        print('Calculating validation map via the fast method.')
        t_start = time.time()
        img_fast = getExplicitValidationImage(
            self.grid.cadImageHeight,
            self.grid.cadImageWidth,
            self.grid.rows,
            self.grid.cols,
            self.grid.radius*self.grid.validationMargin,
            self.grid.useColumnOffsets,
            True,
            self.grid.kaiserRightColumnOffset,
            self.grid.columnOffsets
            )
        t_fast = time.time()-t_start
        print(f'Fast method took {t_fast:.2f}s')

        self.assertEqual(img_slow.shape, img_fast.shape, "Validation maps shapes differ")
        self.assertEqual(img_slow.all(), img_fast.all(), "Validation maps values differ")

if __name__ == '__main__':
    # import rostest
    # rostest.rosrun(PKG, 'test_bare_bones', TestBareBones)

    unittest.main()