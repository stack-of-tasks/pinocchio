import unittest
import math

import numpy as np
import pinocchio as se3
from pinocchio.utils import rand, zero, eye
from pinocchio.explog import exp, log

from test_case import TestCase


class TestExpLog(TestCase):
    def test_exp3(self):
        v = zero(3)
        m = se3.exp3(v)
        self.assertApprox(m, eye(3))

    def test_Jexp3(self):
        v = zero(3)
        m = se3.Jexp3(v)
        self.assertApprox(m, eye(3))

    def test_log3(self):
        m = eye(3)
        v = se3.log3(m)
        self.assertApprox(v, zero(3))

    def test_Jlog3(self):
        m = eye(3)
        J = se3.Jlog3(m)
        self.assertApprox(J, eye(3))

    def test_exp6(self):
        v = se3.Motion.Zero()
        m = se3.exp6(v)
        self.assertTrue(m.isIdentity())

    def test_Jexp6(self):
        v = se3.Motion.Zero()
        J = se3.Jexp6(v)
        self.assertApprox(J,eye(6))
    
    def test_log6(self):
        m = se3.SE3.Identity()
        v = se3.log6(m)
        self.assertApprox(v.vector, zero(6))
    
    def test_log6_homogeneous(self):
        m = eye(4)
        v = se3.log6(m)
        self.assertApprox(v.vector, zero(6))
    
    def test_Jlog6(self):
        m = se3.SE3.Identity()
        J = se3.Jlog6(m)
        self.assertApprox(J, eye(6))

    def test_explog(self):
        self.assertApprox(exp(42), math.exp(42))
        self.assertApprox(log(42), math.log(42))
        self.assertApprox(exp(log(42)), 42)
        self.assertApprox(log(exp(42)), 42)
        m = rand(3)
        self.assertTrue(np.linalg.norm(m) < np.pi) # necessary for next test
        self.assertApprox(log(exp(m)), m)
        m = se3.SE3.Random()
        self.assertApprox(exp(log(m)), m)
        m = rand(6)
        self.assertTrue(np.linalg.norm(m) < np.pi) # necessary for next test (actually, only angular part)
        self.assertApprox(log(exp(m)), m)
        m = eye(4)
        self.assertApprox(exp(log(m)).homogeneous, m)
        with self.assertRaises(ValueError):
            exp(eye(4))
        with self.assertRaises(ValueError):
            exp(list(range(3)))
        with self.assertRaises(ValueError):
            log(list(range(3)))
        with self.assertRaises(ValueError):
            log(zero(5))
        with self.assertRaises(ValueError):
            log(zero((3,1)))


if __name__ == '__main__':
    unittest.main()
