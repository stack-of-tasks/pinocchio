import unittest
from test_case import TestCase
import pinocchio as se3
import pinocchio.utils as utils
import numpy as np
import os

# test urdf
current_file = os.path.dirname(os.path.abspath(__file__))
pinocchio_models_dir = os.path.abspath(os.path.join(current_file, '../models'))
filename = os.path.abspath(os.path.join(pinocchio_models_dir, 'simple_humanoid.urdf'))

# common quantities for all tests
model = se3.buildModelFromUrdf(filename,se3.JointModelFreeFlyer())

qmax = np.matrix(np.full((model.nv,1),np.pi))
q = se3.randomConfiguration(model,-qmax,qmax)
v = utils.rand(model.nv)
tau = utils.rand(model.nv)

J = se3.jointJacobian(model,model.createData(),q,model.getJointId('LLEG_ANKLE_R'),se3.ReferenceFrame.LOCAL,True)
gamma = utils.zero(6)

r_coeff = 0.0
inv_damping = 0.0
update_kinematics = True

# test case
class TestMotionBindings(TestCase):

    def test_forwardDynamics7(self):
        data = model.createData()
        ddq = se3.forwardDynamics(model,data,q,v,tau,J,gamma)
        self.assertFalse(np.isnan(ddq).any())

    def test_forwardDynamics8(self):
        data = model.createData()
        ddq = se3.forwardDynamics(model,data,q,v,tau,J,gamma,r_coeff)
        self.assertFalse(np.isnan(ddq).any())

    def test_forwardDynamics9(self):
        data = model.createData()
        ddq = se3.forwardDynamics(model,data,q,v,tau,J,gamma,r_coeff,update_kinematics)
        self.assertFalse(np.isnan(ddq).any())

    def test_forwardDynamics789(self):
        data7 = model.createData()
        data8 = model.createData()
        data9 = model.createData()
        ddq7 = se3.forwardDynamics(model,data7,q,v,tau,J,gamma)
        ddq8 = se3.forwardDynamics(model,data8,q,v,tau,J,gamma,r_coeff)
        ddq9 = se3.forwardDynamics(model,data9,q,v,tau,J,gamma,r_coeff,update_kinematics)
        self.assertTrue((ddq7==ddq8).all())
        self.assertTrue((ddq7==ddq9).all())
        self.assertTrue((ddq8==ddq9).all())

    def test_impulseDynamics5(self):
        data = model.createData()
        ddq = se3.impulseDynamics(model,data,q,v,J)
        self.assertFalse(np.isnan(ddq).any())

    def test_impulseDynamics6(self):
        data = model.createData()
        ddq = se3.impulseDynamics(model,data,q,v,J,inv_damping)
        self.assertFalse(np.isnan(ddq).any())

    def test_impulseDynamics7(self):
        data = model.createData()
        ddq = se3.impulseDynamics(model,data,q,v,J,inv_damping,update_kinematics)
        self.assertFalse(np.isnan(ddq).any())

    def test_impulseDynamics567(self):
        data5 = model.createData()
        data6 = model.createData()
        data7 = model.createData()
        vnext5 = se3.impulseDynamics(model,data5,q,v,J)
        vnext6 = se3.impulseDynamics(model,data6,q,v,J,inv_damping)
        vnext7 = se3.impulseDynamics(model,data7,q,v,J,inv_damping,update_kinematics)
        self.assertTrue((vnext5==vnext6).all())
        self.assertTrue((vnext5==vnext7).all())
        self.assertTrue((vnext6==vnext7).all())

if __name__ == '__main__':
    unittest.main()
