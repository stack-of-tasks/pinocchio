import unittest
from math import pi

import numpy as np
from random import random

import pinocchio as pin
from pinocchio.utils import npToTuple
from pinocchio.rpy import matrixToRpy, rpyToMatrix, rotate, rpyToJac

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

    def test_rpyToJac(self):
        # Check identity at zero
        rpy = np.zeros(3)
        j0 = rpyToJac(rpy)
        self.assertTrue((j0 == np.eye(3)).all())
        jL = rpyToJac(rpy, pin.LOCAL)
        self.assertTrue((jL == np.eye(3)).all())
        jW = rpyToJac(rpy, pin.WORLD)
        self.assertTrue((jW == np.eye(3)).all())
        jA = rpyToJac(rpy, pin.LOCAL_WORLD_ALIGNED)
        self.assertTrue((jA == np.eye(3)).all())

        # Check correct identities between different versions
        r = random()*2*pi - pi
        p = random()*pi - pi/2
        y = random()*2*pi - pi
        rpy = np.array([r, p, y])
        R = rpyToMatrix(rpy)
        j0 = rpyToJac(rpy)
        jL = rpyToJac(rpy, pin.LOCAL)
        jW = rpyToJac(rpy, pin.WORLD)
        jA = rpyToJac(rpy, pin.LOCAL_WORLD_ALIGNED)
        self.assertTrue((j0 == jL).all())
        self.assertTrue((jW == jA).all())
        self.assertApprox(jW, R.dot(jL))

        # Check against finite differences
        rpydot = np.random.rand(3)
        eps = 1e-7
        tol = 1e-5

        dRdr = (rpyToMatrix(r + eps, p, y) - R) / eps
        dRdp = (rpyToMatrix(r, p + eps, y) - R) / eps
        dRdy = (rpyToMatrix(r, p, y + eps) - R) / eps
        Rdot = dRdr * rpydot[0] + dRdp * rpydot[1] + dRdy * rpydot[2]

        omegaL = jL.dot(rpydot)
        self.assertApprox(Rdot, R.dot(pin.skew(omegaL)), tol)

        omegaW = jW.dot(rpydot)
        self.assertApprox(Rdot, pin.skew(omegaW).dot(R), tol)

if __name__ == '__main__':
    unittest.main()
