from math import pi

import numpy as np
from pinocchio.rpy import matrixToRpy, npToTuple, rotate, rpyToMatrix

from test_case import TestCase


class TestRPY(TestCase):
    def test_npToTuple(self):
        m = np.matrix(list(range(9)))
        self.assertEqual(npToTuple(m), tuple(range(9)))
        self.assertEqual(npToTuple(m.T), tuple(range(9)))
        self.assertEqual(npToTuple(np.reshape(m, (3, 3))), ((0, 1, 2), (3, 4, 5), (6, 7, 8)))

    def test_rotate(self):
        self.assertApprox(rotate('x', pi / 2), np.matrix('1. 0. 0.;0. 0. -1.;0. 1. 0.'))
        self.assertApprox(rotate('x', pi) * rotate('y', pi), rotate('z', pi))
        m = rotate('x', pi / 3) * rotate('y', pi / 5) * rotate('y', pi / 7)
        self.assertApprox(rpyToMatrix(matrixToRpy(m)), m)
        rpy = np.matrix(list(range(3))).T * pi / 2
        self.assertApprox(matrixToRpy(rpyToMatrix(rpy)), rpy)

if __name__ == '__main__':
    unittest.main()
