import unittest
from test_case import TestCase
import pinocchio as pin
from pinocchio.utils import rand, zero
import numpy as np

# common quantities for all tests.
# They correspond to the default values of the arguments, and they need to stay this way
r_coeff = 0.0
inv_damping = 0.0
update_kinematics = True


class TestComBindings(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.matrix(np.full((self.model.nq,1),np.pi))
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)

    def test_mass(self):
        mass = pin.computeTotalMass(self.model)
        self.assertIsNot(mass, np.nan)

        mass_check = sum([inertia.mass for inertia in self.model.inertias[1:] ])
        self.assertApprox(mass,mass_check)

        mass_data = pin.computeTotalMass(self.model,self.data)
        self.assertIsNot(mass_data, np.nan)
        self.assertApprox(mass,mass_data)
        self.assertApprox(mass_data,self.data.mass[0])

        data2 = self.model.createData()
        pin.centerOfMass(self.model,data2,self.q)
        self.assertApprox(mass,data2.mass[0])

    def test_subtree_masses(self):
        pin.computeSubtreeMasses(self.model,self.data)

        data2 = self.model.createData()
        pin.centerOfMass(self.model,data2,self.q)

        for i in range(self.model.njoints):
            self.assertApprox(self.data.mass[i],data2.mass[i])

    def test_Jcom_update3(self):
        Jcom = pin.jacobianCenterOfMass(self.model,self.data,self.q)
        self.assertFalse(np.isnan(Jcom).any())

    def test_Jcom_update4(self):
        Jcom = pin.jacobianCenterOfMass(self.model,self.data,self.q,True)
        self.assertFalse(np.isnan(Jcom).any())
        self.assertFalse(np.isnan(self.data.com[1]).any())

    def test_Jcom_noupdate2(self):
        data_no = self.data
        data_up = self.model.createData()

        pin.forwardKinematics(self.model,data_no,self.q)
        Jcom_no = pin.jacobianCenterOfMass(self.model,data_no)

        Jcom_up = pin.jacobianCenterOfMass(self.model,data_up,self.q)

        self.assertTrue((Jcom_no==Jcom_up).all())

    def test_Jcom_noupdate3(self):
        data_no = self.data
        data_up = self.model.createData()

        pin.forwardKinematics(self.model,data_no,self.q)
        Jcom_no = pin.jacobianCenterOfMass(self.model,data_no,True)

        Jcom_up = pin.jacobianCenterOfMass(self.model,data_up,self.q,True)

        self.assertTrue((Jcom_no==Jcom_up).all())
        self.assertTrue((data_no.com[1]==data_up.com[1]).all())

if __name__ == '__main__':
    unittest.main()
