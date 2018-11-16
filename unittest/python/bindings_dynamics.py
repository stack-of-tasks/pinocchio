import unittest
from test_case import TestCase
import pinocchio as se3
from pinocchio.utils import rand, zero
import numpy as np

# common quantities for all tests.
# They correspond to the default values of the arguments, and they need to stay this way
r_coeff = 0.0
inv_damping = 0.0
update_kinematics = True


class TestDynamicsBindings(TestCase):

    def setUp(self):
        self.model = se3.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.matrix(np.full((self.model.nv,1),np.pi))
        self.q = se3.randomConfiguration(self.model,-qmax,qmax)
        self.v = rand(self.model.nv)
        self.tau = rand(self.model.nv)

        self.v0 = zero(self.model.nv)
        self.tau0 = zero(self.model.nv)
        self.tolerance = 1e-9

        # we compute J on a different self.data
        self.J = se3.jointJacobian(self.model,self.model.createData(),self.q,self.model.getJointId('lleg6_joint'),se3.ReferenceFrame.LOCAL,True)
        self.gamma = zero(6)

    def test_forwardDynamics7(self):
        self.model.gravity = se3.Motion.Zero()
        ddq = se3.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

    def test_forwardDynamics8(self):
        self.model.gravity = se3.Motion.Zero()
        ddq = se3.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma,r_coeff)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

    def test_forwardDynamics9(self):
        self.model.gravity = se3.Motion.Zero()
        ddq = se3.forwardDynamics(self.model,self.data,self.q,self.v0,self.tau0,self.J,self.gamma,r_coeff,update_kinematics)
        self.assertLess(np.linalg.norm(ddq), self.tolerance)

    def test_forwardDynamics789(self):
        data7 = self.data
        data8 = self.model.createData()
        data9 = self.model.createData()
        ddq7 = se3.forwardDynamics(self.model,data7,self.q,self.v,self.tau,self.J,self.gamma)
        ddq8 = se3.forwardDynamics(self.model,data8,self.q,self.v,self.tau,self.J,self.gamma,r_coeff)
        ddq9 = se3.forwardDynamics(self.model,data9,self.q,self.v,self.tau,self.J,self.gamma,r_coeff,update_kinematics)
        self.assertTrue((ddq7==ddq8).all())
        self.assertTrue((ddq7==ddq9).all())
        self.assertTrue((ddq8==ddq9).all())

    def test_impulseDynamics5(self):
        vnext = se3.impulseDynamics(self.model,self.data,self.q,self.v0,self.J)
        self.assertLess(np.linalg.norm(vnext), self.tolerance)

    def test_impulseDynamics6(self):
        vnext = se3.impulseDynamics(self.model,self.data,self.q,self.v0,self.J,inv_damping)
        self.assertLess(np.linalg.norm(vnext), self.tolerance)

    def test_impulseDynamics7(self):
        vnext = se3.impulseDynamics(self.model,self.data,self.q,self.v0,self.J,inv_damping,update_kinematics)
        self.assertLess(np.linalg.norm(vnext), self.tolerance)

    def test_impulseDynamics567(self):
        data5 = self.data
        data6 = self.model.createData()
        data7 = self.model.createData()
        vnext5 = se3.impulseDynamics(self.model,data5,self.q,self.v,self.J)
        vnext6 = se3.impulseDynamics(self.model,data6,self.q,self.v,self.J,inv_damping)
        vnext7 = se3.impulseDynamics(self.model,data7,self.q,self.v,self.J,inv_damping,update_kinematics)
        self.assertTrue((vnext5==vnext6).all())
        self.assertTrue((vnext5==vnext7).all())
        self.assertTrue((vnext6==vnext7).all())

if __name__ == '__main__':
    unittest.main()
