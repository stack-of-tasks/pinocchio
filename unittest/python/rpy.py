import unittest
from math import pi

import numpy as np
import pinocchio as pin

from pinocchio.utils import npToTuple
from pinocchio.rpy import matrixToRpy, rpyToMatrix, rotate

from test_case import PinocchioTestCase as TestCase


class TestRPY(TestCase):
    def test_npToTuple(self):
        m = np.array(list(range(9)))
        self.assertEqual(npToTuple(m), tuple(range(9)))
        self.assertEqual(npToTuple(m.T), tuple(range(9)))
        self.assertEqual(npToTuple(np.reshape(m, (3, 3))), ((0, 1, 2), (3, 4, 5), (6, 7, 8)))

    def test_rotate(self):
        self.assertApprox(rotate('x', pi / 2), np.array([[1., 0., 0.],[0., 0., -1.],[0., 1., 0.]]))
        self.assertApprox(rotate('x', pi).dot(rotate('y', pi)), rotate('z', pi))
        m = rotate('x', pi / 3).dot(rotate('y', pi / 5)).dot(rotate('y', pi / 7))
        self.assertApprox(rpyToMatrix(matrixToRpy(m)), m)
        rpy = np.array(list(range(3))) * pi / 2
        self.assertApprox(matrixToRpy(rpyToMatrix(rpy)), rpy)
        self.assertApprox(rpyToMatrix(rpy), rpyToMatrix(float(rpy[0]), float(rpy[1]), float(rpy[2])))

        try:
            rotate('toto',10.)
        except ValueError:
          self.assertTrue(True)
        else:
          self.assertTrue(False)

        try:
            rotate('w',10.)
        except ValueError:
          self.assertTrue(True)
        else:
          self.assertTrue(False)

if __name__ == '__main__':
    unittest.main()
