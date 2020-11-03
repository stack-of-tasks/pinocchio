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

    def test_centroidal_derivatives(self):

        res = pin.computeCentroidalDynamicsDerivatives(self.model,self.data,self.q,self.v,self.a)

        self.assertTrue(len(res) == 4)

        data2 = self.model.createData()
        pin.computeCentroidalMomentumTimeVariation(self.model,data2,self.q,self.v,self.a)

        self.assertApprox(self.data.hg,data2.hg)
        self.assertApprox(self.data.dhg,data2.dhg)

        data3 = self.model.createData()
        pin.computeRNEADerivatives(self.model,data3,self.q,self.v,self.a)
        res2 = pin.getCentroidalDynamicsDerivatives(self.model,data3)

        for k in range(4):
            self.assertApprox(res[k],res2[k])

if __name__ == '__main__':
    unittest.main()
