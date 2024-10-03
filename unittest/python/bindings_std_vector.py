import pickle
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


class TestStdMap(TestCase):
    def setUp(self):
        pass

    def test_pickle(self):
        vec = pin.StdVec_Vector3()
        for k in range(100):
            vec.append(np.random.rand(3))

        pickle.dump(vec, Path("save_std_vec.p").open("wb"))

        vec_loaded = pickle.load(Path("save_std_vec.p").open("rb"))
        for k in range(len(vec)):
            self.assertApprox(vec[k], vec_loaded[k])


if __name__ == "__main__":
    unittest.main()
