import unittest
import pinocchio as pin
import numpy as np

from test_case import PinocchioTestCase as TestCase

class TestLiegroupBindings(TestCase):

    def test_basic(self):
        R3 = pin.liegroups.R3()
        SO3 = pin.liegroups.SO3()
        R3xSO3 = R3 * SO3
        R10 = pin.liegroups.Rn(10)

        self.assertEqual(R3.name, "R^3")
        self.assertEqual(R10.name, "R^10")
        self.assertEqual(SO3.name, "SO(3)")
        self.assertEqual(R3xSO3.name, R3.name + " x " + SO3.name)

        self.assertEqual(R3.nq+SO3.nq, R3xSO3.nq)
        self.assertEqual(R3.nv+SO3.nv, R3xSO3.nv)

    def test_dIntegrate(self):
        SELF = 0

        for lg in [ pin.liegroups.R3(),
                    pin.liegroups.SO3(),
                    pin.liegroups.SE3(),
                    pin.liegroups.R3() * pin.liegroups.SO3(),
                    ]:
            q = lg.random()
            v = np.random.rand(lg.nv)

            q_int = lg.integrate(q,v)

            q_interpolate = lg.interpolate(q,q_int,0.5)

            v_diff = lg.difference(q,q_int)
            self.assertApprox(v,v_diff)

            J = lg.dIntegrate_dq(q, v)

            J0 = np.random.rand(lg.nv,lg.nv)

            J1 = lg.dIntegrate_dq(q, v, SELF, J0)
            self.assertTrue(np.allclose(np.dot(J, J0),J1))

            J1 = lg.dIntegrate_dq(q, v, J0, SELF)
            self.assertTrue(np.allclose(np.dot(J0, J),J1))

            J = lg.dIntegrate_dv(q, v)

            J0 = np.random.rand(lg.nv,lg.nv)

            J1 = lg.dIntegrate_dv(q, v, SELF, J0)
            self.assertTrue(np.allclose(np.dot(J, J0),J1))

            J1 = lg.dIntegrate_dv(q, v, J0, SELF)
            self.assertTrue(np.allclose(np.dot(J0, J),J1))

    def test_dDifference(self):
        for lg in [ pin.liegroups.R3(),
                    pin.liegroups.SO3(),
                    pin.liegroups.SE3(),
                    pin.liegroups.R3() * pin.liegroups.SO3(),
                    ]:
            q0 = lg.random()
            q1 = lg.random()

            for arg in [ pin.ARG0, pin.ARG1 ]:
                J = lg.dDifference(q0, q1, arg)

                SELF = 0
                J0 = np.random.rand(lg.nv,lg.nv)


                J1 = lg.dDifference(q0, q1, arg, SELF, J0)
                self.assertTrue(np.allclose(np.dot(J, J0),J1))

                J1 = lg.dDifference(q0, q1, arg, J0, SELF)
                self.assertTrue(np.allclose(np.dot(J0, J),J1))

    def test_dIntegrateTransport(self):
        for lg in [ pin.liegroups.R3(),
                    pin.liegroups.SO3(),
                    pin.liegroups.SE3(),
                    pin.liegroups.R3() * pin.liegroups.SO3(),
                    ]:
            q = lg.random()
            v = np.random.rand(lg.nv)

            for arg in [ pin.ARG0, pin.ARG1 ]:
            
                Jint = lg.dIntegrate(q, v, arg)
                J0 = np.random.rand(lg.nv,lg.nv)
                Jout1 = lg.dIntegrateTransport(q, v, J0, arg)
                Jout1_ref = Jint.dot(J0)
                self.assertApprox(Jout1,Jout1_ref)

if __name__ == '__main__':
    unittest.main()
