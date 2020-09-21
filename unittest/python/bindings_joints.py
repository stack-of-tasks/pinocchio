import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin
import numpy as np

class TestJointsAlgo(TestCase):

    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()

        qmax = np.full((self.model.nq,1),np.pi)
        self.q = pin.randomConfiguration(self.model,-qmax,qmax)
        self.v = np.random.rand(self.model.nv)

    def test_basic(self):
        model = self.model

        q_rand = np.random.rand((model.nq))
        q_rand = pin.normalize(model,q_rand)

        self.assertTrue(abs(np.linalg.norm(q_rand[3:7])-1.) <= 1e-8)

        q_next = pin.integrate(model,self.q,np.zeros((model.nv)))
        self.assertApprox(q_next,self.q)

        v_diff = pin.difference(model,self.q,q_next)
        self.assertApprox(v_diff,np.zeros((model.nv)))

        q_next = pin.integrate(model,self.q,self.v)
        q_int = pin.interpolate(model,self.q,q_next,0.5)

        self.assertApprox(q_int,q_int)

        value = pin.squaredDistance(model,self.q,self.q)
        self.assertTrue((value <= 1e-8).all())

        dist = pin.distance(model,self.q,self.q)
        self.assertTrue(dist <= 1e-8)

        q_neutral = pin.neutral(model)
        self.assertApprox(q_neutral,q_neutral)

        q_rand1 = pin.randomConfiguration(model)
        q_rand2 = pin.randomConfiguration(model,-np.ones((model.nq)),np.ones((model.nq)))

        self.assertTrue(pin.isSameConfiguration(model,self.q,self.q,1e-8))

        self.assertFalse(pin.isSameConfiguration(model,q_rand1,q_rand2,1e-8))

    def test_derivatives(self):
        model = self.model

        q = self.q
        v = self.v

        J0, J1 = pin.dIntegrate(model,q,v)
        res_0 = pin.dIntegrate(model,q,v,pin.ARG0)
        res_1 = pin.dIntegrate(model,q,v,pin.ARG1)

        self.assertApprox(J0,res_0)
        self.assertApprox(J1,res_1)

        q_next = pin.integrate(model,q,v)

        J0, J1 = pin.dDifference(model,q,q_next)
        res_0 = pin.dDifference(model,q,q_next,pin.ARG0)
        res_1 = pin.dDifference(model,q,q_next,pin.ARG1)

        self.assertApprox(J0,res_0)
        self.assertApprox(J1,res_1)

if __name__ == '__main__':
    unittest.main()
