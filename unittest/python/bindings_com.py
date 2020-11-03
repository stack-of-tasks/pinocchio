import unittest
from test_case import PinocchioTestCase as TestCase
import pinocchio as pin
from pinocchio.utils import rand
import numpy as np

class TestComBindings(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq,1),np.pi)
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

    def test_com_0(self):
        data = self.data

        c0 = pin.centerOfMass(self.model,self.data,self.q)
        c0_bis = pin.centerOfMass(self.model,self.data,self.q,False)

        self.assertApprox(c0,c0_bis)

        data2 = self.model.createData()
        pin.forwardKinematics(self.model,data,self.q)
        c0 = pin.centerOfMass(self.model,data,pin.POSITION)
        pin.forwardKinematics(self.model,data2,self.q)
        c0_bis = pin.centerOfMass(self.model,data2,pin.POSITION,False)

        self.assertApprox(c0,c0_bis)

        c0_bis = pin.centerOfMass(self.model,self.data,0)

        self.assertApprox(c0,c0_bis)

        self.assertApprox(c0,data2.com[0])
        self.assertApprox(self.data.com[0],data2.com[0])

    def test_com_1(self):
        data = self.data

        v = rand(self.model.nv)
        c0 = pin.centerOfMass(self.model,self.data,self.q,v)
        c0_bis = pin.centerOfMass(self.model,self.data,self.q,v,False)

        self.assertApprox(c0,c0_bis)

        data2 = self.model.createData()
        pin.forwardKinematics(self.model,data,self.q,v)
        c0 = pin.centerOfMass(self.model,data,pin.VELOCITY)
        pin.forwardKinematics(self.model,data2,self.q,v)
        c0_bis = pin.centerOfMass(self.model,data2,pin.VELOCITY,False)

        self.assertApprox(c0,c0_bis)

        c0_bis = pin.centerOfMass(self.model,data2,1)

        self.assertApprox(c0,c0_bis)

        data3 = self.model.createData()
        pin.centerOfMass(self.model,data3,self.q)

        self.assertApprox(self.data.com[0],data2.com[0])
        self.assertApprox(self.data.vcom[0],data2.vcom[0])

        self.assertApprox(self.data.com[0],data3.com[0])

    def test_com_2(self):
        data = self.data

        v = rand(self.model.nv)
        a = rand(self.model.nv)
        c0 = pin.centerOfMass(self.model,self.data,self.q,v,a)
        c0_bis = pin.centerOfMass(self.model,self.data,self.q,v,a,False)

        self.assertApprox(c0,c0_bis)

        data2 = self.model.createData()
        pin.forwardKinematics(self.model,data,self.q,v,a)
        c0 = pin.centerOfMass(self.model,data,pin.ACCELERATION)
        pin.forwardKinematics(self.model,data2,self.q,v,a)
        c0_bis = pin.centerOfMass(self.model,data2,pin.ACCELERATION,False)

        self.assertApprox(c0,c0_bis)

        c0_bis = pin.centerOfMass(self.model,data2,2)
        self.assertApprox(c0,c0_bis)

        data3 = self.model.createData()
        pin.centerOfMass(self.model,data3,self.q)

        data4 = self.model.createData()
        pin.centerOfMass(self.model,data4,self.q,v)

        self.assertApprox(self.data.com[0],data2.com[0])
        self.assertApprox(self.data.vcom[0],data2.vcom[0])
        self.assertApprox(self.data.acom[0],data2.acom[0])

        self.assertApprox(self.data.com[0],data3.com[0])

        self.assertApprox(self.data.com[0],data4.com[0])
        self.assertApprox(self.data.vcom[0],data4.vcom[0])

    def test_com_default(self):
        v = rand(self.model.nv)
        a = rand(self.model.nv)
        pin.centerOfMass(self.model,self.data,self.q,v,a)

        data2 = self.model.createData()
        pin.forwardKinematics(self.model,data2,self.q,v,a)
        pin.centerOfMass(self.model,data2)

        for i in range(self.model.njoints):
            self.assertApprox(self.data.com[i],data2.com[i])
            self.assertApprox(self.data.vcom[i],data2.vcom[i])
            self.assertApprox(self.data.acom[i],data2.acom[i])

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

    def test_subtree_jacobian(self):
        model = self.model
        data = self.data

        Jcom = pin.jacobianCenterOfMass(model,data,self.q)
        Jcom_subtree = pin.getJacobianSubtreeCenterOfMass(model,data,0)
        self.assertApprox(Jcom,Jcom_subtree)

        data2 = model.createData()
        Jcom_subtree2 = pin.jacobianSubtreeCenterOfMass(model,data2,self.q,0)
        self.assertApprox(Jcom_subtree,Jcom_subtree2)

        data3 = model.createData()
        Jcom_subtree3 = pin.jacobianSubtreeCoMJacobian(model,data3,self.q,0)
        self.assertApprox(Jcom_subtree3,Jcom_subtree2)

        Jcom_subtree4 = pin.jacobianSubtreeCoMJacobian(model,data3,0)
        self.assertApprox(Jcom_subtree3,Jcom_subtree4)

if __name__ == '__main__':
    unittest.main()
