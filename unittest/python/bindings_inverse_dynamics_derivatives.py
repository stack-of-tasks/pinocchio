import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin
import numpy as np

class TestDeriavtives(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq,1),np.pi)
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)
        self.v = np.random.rand((self.model.nv))
        self.a = np.random.rand((self.model.nv))

        self.fext = []
        for _ in range(self.model.njoints):
          self.fext.append(pin.Force.Random())

    def test_rnea_derivatives(self):

        res = pin.computeRNEADerivatives(self.model,self.data,self.q,self.v,self.a)

        self.assertTrue(len(res) == 3)

        data2 = self.model.createData()
        pin.rnea(self.model,data2,self.q,self.v,self.a)

        self.assertApprox(self.data.ddq,data2.ddq)

        # With external forces
        res = pin.computeRNEADerivatives(self.model,self.data,self.q,self.v,self.a,self.fext)

        self.assertTrue(len(res) == 3)

        pin.rnea(self.model,data2,self.q,self.v,self.a,self.fext)

        self.assertApprox(self.data.ddq,data2.ddq)

    def test_generalized_gravity_derivatives(self):

        res = pin.computeGeneralizedGravityDerivatives(self.model,self.data,self.q)

        data2 = self.model.createData()
        ref,_,_ = pin.computeRNEADerivatives(self.model,data2,self.q,self.v*0,self.a*0)

        self.assertApprox(res,ref)

    def test_static_torque_derivatives(self):

        res = pin.computeStaticTorqueDerivatives(self.model,self.data,self.q,self.fext)

        data2 = self.model.createData()
        ref,_,_ = pin.computeRNEADerivatives(self.model,data2,self.q,self.v*0,self.a*0,self.fext)

        self.assertApprox(res,ref)

if __name__ == '__main__':
    unittest.main()
