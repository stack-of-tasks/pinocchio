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
        map = pin.StdMap_String_VectorXd()
        keys = []
        for k in range(100):
            key_name = "key_" + str(k + 1)
            keys.append(key_name)
            map[key_name] = np.random.rand(10)

        pickle.dump(map, Path("save_std_map.p").open("wb"))

        map_loaded = pickle.load(Path("save_std_map.p").open("rb"))
        for key in keys:
            self.assertApprox(map[key], map_loaded[key])


if __name__ == "__main__":
    unittest.main()
