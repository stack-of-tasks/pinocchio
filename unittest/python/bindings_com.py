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


class TestComBindings(TestCase):

    def setUp(self):
        self.model = se3.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.matrix(np.full((self.model.nv,1),np.pi))
        self.q = se3.randomConfiguration(self.model,-qmax,qmax)

    def test_Jcom_update3(self):
        Jcom = se3.jacobianCenterOfMass(self.model,self.data,self.q)
        self.assertFalse(np.isnan(Jcom).any())

    def test_Jcom_update4(self):
        Jcom = se3.jacobianCenterOfMass(self.model,self.data,self.q,True)
        self.assertFalse(np.isnan(Jcom).any())
        self.assertFalse(np.isnan(self.data.com[1]).any())

    def test_Jcom_noupdate2(self):
        data_no = self.data
        data_up = self.model.createData()

        se3.forwardKinematics(self.model,data_no,self.q)
        Jcom_no = se3.jacobianCenterOfMass(self.model,data_no)

        Jcom_up = se3.jacobianCenterOfMass(self.model,data_up,self.q)

        self.assertTrue((Jcom_no==Jcom_up).all())

    def test_Jcom_noupdate3(self):
        data_no = self.data
        data_up = self.model.createData()

        se3.forwardKinematics(self.model,data_no,self.q)
        Jcom_no = se3.jacobianCenterOfMass(self.model,data_no,True)

        Jcom_up = se3.jacobianCenterOfMass(self.model,data_up,self.q,True)

        self.assertTrue((Jcom_no==Jcom_up).all())
        self.assertTrue((data_no.com[1]==data_up.com[1]).all())

if __name__ == '__main__':
    unittest.main()
