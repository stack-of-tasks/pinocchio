import unittest
from test_case import PinocchioTestCase as TestCase
import pinocchio as pin
from pinocchio.utils import rand, zero
import numpy as np

import warnings

# common quantities for all tests.
# They correspond to the default values of the arguments, and they need to stay this way
r_coeff = 0.0
inv_damping = 0.0


class TestDynamicsBindings(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq,1),np.pi)
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)
        self.v = rand(self.model.nv)
        self.tau = rand(self.model.nv)

        self.v0 = zero(self.model.nv)
        self.tau0 = zero(self.model.nv)
        self.tolerance = 1e-9

        # we compute J on a different self.data
        self.J = pin.computeJointJacobian(self.model,self.model.createData(),self.q,self.model.getJointId('lleg6_joint'))
        self.gamma = zero(6)

    def test_forwardDynamics_default(self):
        data_no_q = self.model.createData()

        self.model.gravity = pin.Motion.Zero()
        ddq = pin.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

        KKT_inverse = pin.getKKTContactDynamicMatrixInverse(self.model,self.data,self.J)
        M = pin.crba(self.model,self.model.createData(),self.q)

        self.assertApprox(M,np.linalg.inv(KKT_inverse)[:self.model.nv,:self.model.nv])

        pin.computeAllTerms(self.model,data_no_q,self.q,self.v0)
        ddq_no_q = pin.forwardDynamics(self.model,data_no_q,self.tau0,self.J,self.gamma)
        self.assertLess(np.linalg.norm(ddq_no_q), self.tolerance)

        self.assertApprox(ddq,ddq_no_q)

    def test_forwardDynamics_rcoeff(self):
        data_no_q = self.model.createData()

        self.model.gravity = pin.Motion.Zero()
        ddq = pin.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma,r_coeff)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

        pin.computeAllTerms(self.model,data_no_q,self.q,self.v0)
        ddq_no_q = pin.forwardDynamics(self.model,data_no_q,self.tau0,self.J,self.gamma,r_coeff)
        self.assertLess(np.linalg.norm(ddq_no_q), self.tolerance)

        self.assertApprox(ddq,ddq_no_q)

    def test_forwardDynamics_q(self):
        data7 = self.data
        data8 = self.model.createData()
        data9_deprecated = self.model.createData()
        ddq7 = pin.forwardDynamics(self.model,data7,self.q,self.v,self.tau,self.J,self.gamma)
        ddq8 = pin.forwardDynamics(self.model,data8,self.q,self.v,self.tau,self.J,self.gamma,r_coeff)
        with warnings.catch_warnings(record=True) as warning_list:
            ddq9_deprecated = pin.forwardDynamics(self.model,data9_deprecated,self.q,self.v,self.tau,self.J,self.gamma,r_coeff,True)
            self.assertTrue(any(item.category == pin.DeprecatedWarning for item in warning_list))
        self.assertTrue((ddq7==ddq8).all())
        self.assertTrue((ddq7==ddq9_deprecated).all())

    def test_forwardDynamics_no_q(self):
        data5 = self.data
        data6 = self.model.createData()
        data9_deprecated = self.model.createData()
        pin.computeAllTerms(self.model,data5,self.q,self.v0)
        pin.computeAllTerms(self.model,data6,self.q,self.v0)
        pin.computeAllTerms(self.model,data9_deprecated,self.q,self.v0)
        ddq5 = pin.forwardDynamics(self.model,data5,self.tau,self.J,self.gamma)
        ddq6 = pin.forwardDynamics(self.model,data6,self.tau,self.J,self.gamma,r_coeff)
        with warnings.catch_warnings(record=True) as warning_list:
            ddq9_deprecated = pin.forwardDynamics(self.model,data9_deprecated,self.q,self.v,self.tau,self.J,self.gamma,r_coeff,False)
            self.assertTrue(any(item.category == pin.DeprecatedWarning for item in warning_list))
        self.assertTrue((ddq5==ddq6).all())
        self.assertTrue((ddq5==ddq9_deprecated).all())

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
