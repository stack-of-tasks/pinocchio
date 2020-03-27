import unittest
from math import sqrt

import numpy as np
import pinocchio as pin
from pinocchio.utils import isapprox

from test_case import PinocchioTestCase as TestCase

class TestUtils(TestCase):
    def test_se3ToXYZQUAT_XYZQUATToSe3(self):
        m = pin.SE3.Identity()
        m.translation = np.array([1., 2., 3.])
        m.rotation = np.array([[1., 0., 0.],[0., 0., -1.],[0., 1., 0.]])  # rotate('x', pi / 2)
        self.assertApprox(pin.SE3ToXYZQUAT(m).T, [1., 2., 3., sqrt(2) / 2, 0, 0, sqrt(2) / 2])
        self.assertApprox(pin.XYZQUATToSE3([1., 2., 3., sqrt(2) / 2, 0, 0, sqrt(2) / 2]), m)

    def test_isapprox(self):
        self.assertFalse(isapprox(1, 2))
        self.assertTrue(isapprox(1, 2, 10))
        self.assertFalse(isapprox([1e10, 1e-7], [1.00001e10, 1e-8]))
        self.assertTrue(isapprox([1e10, 1e-8], [1.00001e10, 1e-9], 1e-5))

if __name__ == '__main__':
    unittest.main()
