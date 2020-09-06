import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin
import numpy as np

class TestABA(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq,1),np.pi)
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)
        self.v = np.random.rand(self.model.nv)
        self.ddq = np.random.rand(self.model.nv)

        self.fext = []
        for _ in range(self.model.njoints):
          self.fext.append(pin.Force.Random())

    def test_aba(self):
        model = self.model
        ddq = pin.aba(self.model,self.data,self.q,self.v,self.ddq)

        null_fext = pin.StdVec_Force()
        for _ in range(model.njoints):
          null_fext.append(pin.Force.Zero())

        ddq_null_fext = pin.aba(self.model,self.data,self.q,self.v,self.ddq,null_fext)
        self.assertApprox(ddq_null_fext,ddq)

        null_fext_list = []
        for f in null_fext:
            null_fext_list.append(f)

        print('size:',len(null_fext_list))
        ddq_null_fext_list = pin.aba(self.model,self.data,self.q,self.v,self.ddq,null_fext_list)
        self.assertApprox(ddq_null_fext_list,ddq)

    def test_computeMinverse(self):
        model = self.model
        Minv = pin.computeMinverse(model,self.data,self.q)

        data2 = model.createData()
        M = pin.crba(model,data2,self.q)

        self.assertApprox(np.linalg.inv(M),Minv)

if __name__ == '__main__':
    unittest.main()
