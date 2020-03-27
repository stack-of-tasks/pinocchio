import unittest

import numpy as np
from numpy.random import rand

import pinocchio as pin
pin.switchToNumpyArray()
from pinocchio import skew, unSkew

from test_case import PinocchioTestCase

class TestSpatial(PinocchioTestCase):
    def test_skew(self):
        v3 = rand(3)
        self.assertApprox(v3, unSkew(skew(v3)))
        self.assertLess(np.linalg.norm(skew(v3).dot(v3)), 1e-10)

        x, y, z = tuple(rand(3).tolist())
        M = np.array([[ 0.,  x, y],
                      [-x,  0., z],
                      [-y, -z, 0.]])
        self.assertApprox(M, skew(unSkew(M)))

        rhs = rand(3)
        self.assertApprox(np.cross(v3,rhs,axis=0), skew(v3).dot(rhs))
        self.assertApprox(M.dot(rhs), np.cross(unSkew(M),rhs,axis=0))

        x, y, z = tuple(v3.tolist())
        self.assertApprox(skew(v3), np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]]))

if __name__ == '__main__':
    unittest.main()
