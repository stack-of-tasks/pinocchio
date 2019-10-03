import unittest
from test_case import TestCase
import pinocchio as pin
pin.switchToNumpyMatrix()
from pinocchio.utils import rand, zero
import numpy as np

import warnings

# common quantities for all tests.
# They correspond to the default values of the arguments, and they need to stay this way
r_coeff = 0.0
inv_damping = 0.0
update_kinematics = True


class TestDynamicsBindings(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.matrix(np.full((self.model.nq,1),np.pi))
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)
        self.v = rand(self.model.nv)
        self.tau = rand(self.model.nv)

        self.v0 = zero(self.model.nv)
        self.tau0 = zero(self.model.nv)
        self.tolerance = 1e-9

        # we compute J on a different self.data
        self.J = pin.jointJacobian(self.model,self.model.createData(),self.q,self.model.getJointId('lleg6_joint'))
        self.gamma = zero(6)

    def test_forwardDynamics7(self):
        self.model.gravity = pin.Motion.Zero()
        ddq = pin.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

    def test_forwardDynamics8(self):
        self.model.gravity = pin.Motion.Zero()
        ddq = pin.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma,r_coeff)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

    def test_forwardDynamics9(self):
        self.model.gravity = pin.Motion.Zero()
        ddq = pin.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma,r_coeff,update_kinematics)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

    def test_forwardDynamics789(self):
        data7 = self.data
        data8 = self.model.createData()
        data9 = self.model.createData()
        ddq7 = pin.forwardDynamics(self.model,data7,self.q,self.v,self.tau,self.J,self.gamma)
        ddq8 = pin.forwardDynamics(self.model,data8,self.q,self.v,self.tau,self.J,self.gamma,r_coeff)
        ddq9 = pin.forwardDynamics(self.model,data9,self.q,self.v,self.tau,self.J,self.gamma,r_coeff,update_kinematics)
        self.assertTrue((ddq7==ddq8).all())
        self.assertTrue((ddq7==ddq9).all())
        self.assertTrue((ddq8==ddq9).all())

    def test_impulseDynamics_default(self):
        data_no_q = self.model.createData()

        vnext = pin.impulseDynamics(self.model,self.data,self.q,self.v0,self.J)
        self.assertLess(np.linalg.norm(vnext), self.tolerance)

        pin.crba(self.model,data_no_q,self.q)
        vnext_no_q = pin.impulseDynamics(self.model,data_no_q,self.v0,self.J)
        self.assertLess(np.linalg.norm(vnext_no_q), self.tolerance)

        self.assertApprox(vnext, vnext_no_q)

    def test_impulseDynamics_r(self):
        data_no_q = self.model.createData()

        vnext = pin.impulseDynamics(self.model,self.data,self.q,self.v0,self.J,r_coeff)
        self.assertLess(np.linalg.norm(vnext), self.tolerance)

        pin.crba(self.model,data_no_q,self.q)
        vnext_no_q = pin.impulseDynamics(self.model,data_no_q,self.v0,self.J,r_coeff)
        self.assertLess(np.linalg.norm(vnext_no_q), self.tolerance)

        self.assertApprox(vnext, vnext_no_q)

    def test_impulseDynamics_rd(self):
        data_no_q = self.model.createData()

        vnext = pin.impulseDynamics(self.model,self.data,self.q,self.v0,self.J,r_coeff,inv_damping)
        self.assertLess(np.linalg.norm(vnext), self.tolerance)

        pin.crba(self.model,data_no_q,self.q)
        vnext_no_q = pin.impulseDynamics(self.model,data_no_q,self.v0,self.J,r_coeff,inv_damping)
        self.assertLess(np.linalg.norm(vnext_no_q), self.tolerance)

        self.assertApprox(vnext, vnext_no_q)

    def test_impulseDynamics_q(self):
        data5 = self.data
        data6 = self.model.createData()
        data7 = self.model.createData()
        data7_deprecated = self.model.createData()
        vnext5 = pin.impulseDynamics(self.model,data5,self.q,self.v,self.J)
        vnext6 = pin.impulseDynamics(self.model,data6,self.q,self.v,self.J,r_coeff)
        vnext7 = pin.impulseDynamics(self.model,data7,self.q,self.v,self.J,r_coeff,inv_damping)
        with warnings.catch_warnings(record=True) as warning_list:
            vnext7_deprecated = pin.impulseDynamics(self.model,data7_deprecated,self.q,self.v,self.J,r_coeff,True)
            self.assertTrue(any(item.category == pin.DeprecatedWarning for item in warning_list))
        self.assertTrue((vnext5==vnext6).all())
        self.assertTrue((vnext5==vnext7).all())
        self.assertTrue((vnext6==vnext7).all())
        self.assertTrue((vnext7_deprecated==vnext7).all())

    def test_impulseDynamics_no_q(self):
        data4 = self.data
        data5 = self.model.createData()
        data6 = self.model.createData()
        data7_deprecated = self.model.createData()
        pin.crba(self.model,data4,self.q)
        pin.crba(self.model,data5,self.q)
        pin.crba(self.model,data6,self.q)
        pin.crba(self.model,data7_deprecated,self.q)
        vnext4 = pin.impulseDynamics(self.model,data4,self.v,self.J)
        vnext5 = pin.impulseDynamics(self.model,data5,self.v,self.J,r_coeff)
        vnext6 = pin.impulseDynamics(self.model,data6,self.v,self.J,r_coeff,inv_damping)
        with warnings.catch_warnings(record=True) as warning_list:
            vnext7_deprecated = pin.impulseDynamics(self.model,data7_deprecated,self.q,self.v,self.J,r_coeff,False)
            self.assertTrue(any(item.category == pin.DeprecatedWarning for item in warning_list))
        self.assertTrue((vnext4==vnext5).all())
        self.assertTrue((vnext4==vnext6).all())
        self.assertTrue((vnext5==vnext6).all())
        self.assertTrue((vnext7_deprecated==vnext6).all())

if __name__ == '__main__':
    unittest.main()
