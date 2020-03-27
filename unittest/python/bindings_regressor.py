import unittest
from test_case import PinocchioTestCase as TestCase
import pinocchio as pin
from pinocchio.utils import rand, zero
import numpy as np

class TestRegressorBindings(TestCase):

    def test_staticRegressor(self):
        model = pin. buildSampleModelHumanoidRandom()

        data = model.createData()
        data_ref = model.createData()

        model.lowerPositionLimit[:7] = -1.
        model.upperPositionLimit[:7] = 1.

        q = pin.randomConfiguration(model)
        pin.computeStaticRegressor(model,data,q)

        phi = zero(4*(model.njoints-1))
        for k in range(1,model.njoints):
            Y = model.inertias[k]
            phi[4*(k-1)] = Y.mass
            phi[4*k-3:4*k] = Y.mass * Y.lever

        static_com_ref = pin.centerOfMass(model,data_ref,q)
        static_com = data.staticRegressor.dot(phi)

        self.assertApprox(static_com, static_com_ref)

    def test_bodyRegressor(self):
        I = pin.Inertia.Random()
        v = pin.Motion.Random()
        a = pin.Motion.Random()

        f = I*a + I.vxiv(v)

        f_regressor = pin.bodyRegressor(v,a).dot(I.toDynamicParameters())
        
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

        f_regressor = pin.jointBodyRegressor(model,data,JOINT_ID).dot(model.inertias[JOINT_ID].toDynamicParameters())

        self.assertApprox(f_regressor, f.vector)

    def test_frameBodyRegressor(self):
        model = pin.buildSampleModelManipulator()

        JOINT_ID = model.njoints - 1

        framePlacement = pin.SE3.Random()
        FRAME_ID = model.addBodyFrame ("test_body", JOINT_ID, framePlacement, -1)

        data = model.createData()

        q = pin.randomConfiguration(model)
        v = pin.utils.rand(model.nv)
        a = pin.utils.rand(model.nv)

        pin.rnea(model,data,q,v,a)

        f = framePlacement.actInv(data.f[JOINT_ID])
        I = framePlacement.actInv(model.inertias[JOINT_ID])

        f_regressor = pin.frameBodyRegressor(model,data,FRAME_ID).dot(I.toDynamicParameters())

        self.assertApprox(f_regressor, f.vector)

    def test_joint_torque_regressor(self):
        model = pin.buildSampleModelHumanoidRandom()
        model.lowerPositionLimit[:7] = -1.
        model.upperPositionLimit[:7] = 1.

        data = model.createData()
        data_ref = model.createData()

        q = pin.randomConfiguration(model)
        v = pin.utils.rand(model.nv)
        a = pin.utils.rand(model.nv)

        pin.rnea(model,data_ref,q,v,a)

        params = zero(10*(model.njoints-1))
        for i in range(1, model.njoints):
            params[(i-1)*10:i*10] = model.inertias[i].toDynamicParameters()

        pin.computeJointTorqueRegressor(model,data,q,v,a)

        tau_regressor = data.jointTorqueRegressor.dot(params)

        self.assertApprox(tau_regressor, data_ref.tau)

if __name__ == '__main__':
    unittest.main()
