import math

import numpy as np
import pinocchio as se3
from pinocchio.explog import exp, log

from test_case import TestCase


class TestExpLog(TestCase):
    def test_explog(self):
        self.assertApprox(exp(42), math.exp(42))
        self.assertApprox(log(42), math.log(42))
        self.assertApprox(exp(log(42)), 42)
        self.assertApprox(log(exp(42)), 42)
        m = np.matrix(range(1, 4), np.double).T
        self.assertApprox(log(exp(m)), m)
        m = se3.SE3.Random()
        self.assertApprox(exp(log(m)), m)
        m = np.matrix([float(i) / 10 for i in range(1, 7)]).T
        self.assertApprox(log(exp(m)), m)
        m = np.eye(4)
        self.assertApprox(exp(log(m)).homogeneous, m)
        with self.assertRaises(ValueError):
            exp(np.eye(4))
        with self.assertRaises(ValueError):
            exp(range(3))
        with self.assertRaises(ValueError):
            log(range(3))
        with self.assertRaises(ValueError):
            log(np.zeros(5))
