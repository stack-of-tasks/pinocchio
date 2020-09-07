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
        self.tau = np.random.rand((self.model.nv))

        self.fext = []
        for _ in range(self.model.njoints):
          self.fext.append(pin.Force.Random())

    def test_aba_derivatives(self):
        res = pin.computeABADerivatives(self.model,self.data,self.q,self.v,self.tau)

        self.assertTrue(len(res) == 3)

        data2 = self.model.createData()
        pin.aba(self.model,data2,self.q,self.v,self.tau)

        self.assertApprox(self.data.ddq,data2.ddq)

        # With external forces
        res = pin.computeABADerivatives(self.model,self.data,self.q,self.v,self.tau,self.fext)

        self.assertTrue(len(res) == 3)

        pin.aba(self.model,data2,self.q,self.v,self.tau,self.fext)

        self.assertApprox(self.data.ddq,data2.ddq)

if __name__ == '__main__':
    unittest.main()
