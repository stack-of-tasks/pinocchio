import math

import numpy as np
import pinocchio as se3
from pinocchio.utils import rand
from pinocchio.explog import exp, log

from test_case import TestCase


class TestExpLog(TestCase):
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
        m = np.eye(4)
        self.assertApprox(exp(log(m)).homogeneous, m)
        with self.assertRaises(ValueError):
            exp(np.eye(4))
        with self.assertRaises(ValueError):
            exp(list(range(3)))
        with self.assertRaises(ValueError):
            log(list(range(3)))
        with self.assertRaises(ValueError):
            log(np.zeros(5))
        with self.assertRaises(ValueError):
            log(np.zeros((3,1)))

if __name__ == '__main__':
    unittest.main()