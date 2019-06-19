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

    def test_jointBodyRegressor(self):
        model = pin.buildSampleModelManipulator()
        data = model.createData()

        JOINT_ID = model.njoints - 1

        q = pin.randomConfiguration(model)
        v = pin.utils.rand(model.nv)
        a = pin.utils.rand(model.nv)

        pin.rnea(model,data,q,v,a)

        f = data.f[JOINT_ID]

        f_regressor = pin.jointBodyRegressor(model,data,JOINT_ID) * model.inertias[JOINT_ID].toDynamicParameters()

        self.assertApprox(f_regressor, f.vector)

if __name__ == '__main__':
    unittest.main()
