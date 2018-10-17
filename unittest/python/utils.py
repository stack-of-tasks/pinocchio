from math import sqrt

import numpy as np
import pinocchio as se3
from pinocchio.utils import (XYZQUATToSe3, cross, isapprox, se3ToXYZQUAT)

from test_case import TestCase


class TestUtils(TestCase):
    def test_cross(self):
        a = np.matrix('2. 0. 0.').T
        b = np.matrix('0. 3. 0.').T
        c = np.matrix('0. 0. 6.').T
        self.assertApprox(cross(a, b), c)

    def test_se3ToXYZQUAT_XYZQUATToSe3(self):
        m = se3.SE3.Identity()
        m.translation = np.matrix('1. 2. 3.').T
        m.rotation = np.matrix('1. 0. 0.;0. 0. -1.;0. 1. 0.')  # rotate('x', pi / 2)
        self.assertApprox(se3ToXYZQUAT(m), [1., 2., 3., sqrt(2) / 2, 0, 0, sqrt(2) / 2])
        self.assertApprox(XYZQUATToSe3([1., 2., 3., sqrt(2) / 2, 0, 0, sqrt(2) / 2]), m)

    def test_isapprox(self):
        self.assertFalse(isapprox(1, 2))
        self.assertTrue(isapprox(1, 2, 10))
        self.assertFalse(isapprox([1e10, 1e-7], [1.00001e10, 1e-8]))
        self.assertTrue(isapprox([1e10, 1e-8], [1.00001e10, 1e-9], 1e-5))

if __name__ == '__main__':
    unittest.main()