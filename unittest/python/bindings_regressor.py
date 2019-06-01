import unittest
from test_case import TestCase
import pinocchio as pin
from pinocchio.utils import rand, zero
import numpy as np

class TestRegressorBindings(TestCase):

    def test_bodyRegressor(self):
        I = pin.Inertia.Random()
        v = pin.Motion.Random()
        a = pin.Motion.Random()

        f = I*a + I.vxiv(v)

        f_regressor = pin.bodyRegressor(v,a) * I.toDynamicParameters()
        
        self.assertApprox(f_regressor, f.vector)

if __name__ == '__main__':
    unittest.main()
