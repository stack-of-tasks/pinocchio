import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin

pin.switchToNumpyArray()

from pinocchio.utils import rand, zero
import numpy as np

class TestRNEA(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq,1),np.pi)
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)
        self.v = np.random.rand(self.model.nv)
        self.a = np.random.rand(self.model.nv)

    def test_rnea(self):
        model = self.model
        tau = pin.rnea(self.model,self.data,self.q,self.v,self.a)

        null_fext = pin.StdVec_Force()
        for k in range(model.njoints):
          null_fext.append(pin.Force.Zero())

        tau_null_fext = pin.rnea(self.model,self.data,self.q,self.v,self.a,null_fext)
        self.assertApprox(tau_null_fext,tau)

        null_fext_list = []
        for f in null_fext:
            null_fext_list.append(f)

        print('size:',len(null_fext_list))
        tau_null_fext_list = pin.rnea(self.model,self.data,self.q,self.v,self.a,null_fext_list)
        self.assertApprox(tau_null_fext_list,tau)

if __name__ == '__main__':
    unittest.main()
