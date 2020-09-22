import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin
import numpy as np

import pickle

class TestStdMap(TestCase):

    def setUp(self):
        pass

    def test_pickle(self):
        vec = pin.StdVec_Vector3()
        for k in range(100):
            vec.append(np.random.rand((3)))

        pickle.dump( vec, open( "save_std_vec.p", "wb" ) )

        vec_loaded = pickle.load( open( "save_std_vec.p", "rb" ) )
        for k in range(len(vec)):
            self.assertApprox(vec[k],vec_loaded[k])

if __name__ == '__main__':
    unittest.main()
