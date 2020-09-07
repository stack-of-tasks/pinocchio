import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin
import numpy as np

class TestRNEA(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq,1),np.pi)
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)
        self.v = np.random.rand(self.model.nv)
        self.a = np.random.rand(self.model.nv)

        self.fext = []
        for _ in range(self.model.njoints):
          self.fext.append(pin.Force.Random())

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

    def test_nle(self):
        model = self.model

        tau = pin.nonLinearEffects(model,self.data,self.q,self.v)

        data2 = model.createData()
        tau_ref = pin.rnea(model,data2,self.q,self.v,self.a*0)

        self.assertApprox(tau,tau_ref)

    def test_generalized_gravity(self):
        model = self.model

        tau = pin.computeGeneralizedGravity(model,self.data,self.q)

        data2 = model.createData()
        tau_ref = pin.rnea(model,data2,self.q,self.v*0,self.a*0)

        self.assertApprox(tau,tau_ref)

    def test_static_torque(self):
        model = self.model

        tau = pin.computeStaticTorque(model,self.data,self.q,self.fext)

        data2 = model.createData()
        tau_ref = pin.rnea(model,data2,self.q,self.v*0,self.a*0,self.fext)

        self.assertApprox(tau,tau_ref)

    def test_coriolis_matrix(self):
        model = self.model

        C = pin.computeCoriolisMatrix(model,self.data,self.q,self.v)

        data2 = model.createData()
        tau_coriolis_ref = pin.rnea(model,data2,self.q,self.v,self.a*0) - pin.rnea(model,data2,self.q,self.v*0,self.a*0)

        self.assertApprox(tau_coriolis_ref,C.dot(self.v))

if __name__ == '__main__':
    unittest.main()
