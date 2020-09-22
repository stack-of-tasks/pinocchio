import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin
import numpy as np

import pickle

class TestStdMap(TestCase):

    def setUp(self):
        pass

    def test_pickle(self):
        map = pin.StdMap_String_VectorXd()
        keys = []
        for k in range(100):
            key_name = 'key_' + str(k+1)
            keys.append(key_name)
            map[key_name] = np.random.rand((10))

        pickle.dump( map, open( "save_std_map.p", "wb" ) )

        map_loaded = pickle.load( open( "save_std_map.p", "rb" ) )
        for key in keys:
            self.assertApprox(map[key],map_loaded[key])

if __name__ == '__main__':
    unittest.main()
