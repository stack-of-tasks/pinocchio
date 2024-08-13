import unittest

import numpy as np
import pinocchio as pin

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

        self.assertEqual(R3.nq + SO3.nq, R3xSO3.nq)
        self.assertEqual(R3.nv + SO3.nv, R3xSO3.nv)

    def test_dIntegrate(self):
        SELF = 0

        for lg in [
            pin.liegroups.R3(),
            pin.liegroups.SO3(),
            pin.liegroups.SO2(),
            pin.liegroups.SE3(),
            pin.liegroups.SE2(),
            pin.liegroups.R3() * pin.liegroups.SO3(),
        ]:
            q = lg.random()
            v = np.random.rand(lg.nv)

            q_int = lg.integrate(q, v)

            _q_interpolate = lg.interpolate(q, q_int, 0.5)

            v_diff = lg.difference(q, q_int)
            self.assertApprox(v, v_diff)

            J = lg.dIntegrate_dq(q, v)

            J0 = np.random.rand(lg.nv, lg.nv)

            J1 = lg.dIntegrate_dq(q, v, SELF, J0)
            self.assertTrue(np.allclose(np.dot(J, J0), J1))

            J1 = lg.dIntegrate_dq(q, v, J0, SELF)
            self.assertTrue(np.allclose(np.dot(J0, J), J1))

            J = lg.dIntegrate_dv(q, v)

            J0 = np.random.rand(lg.nv, lg.nv)

            J1 = lg.dIntegrate_dv(q, v, SELF, J0)
            self.assertTrue(np.allclose(np.dot(J, J0), J1))

            J1 = lg.dIntegrate_dv(q, v, J0, SELF)
            self.assertTrue(np.allclose(np.dot(J0, J), J1))

    def test_dDifference(self):
        for lg in [
            pin.liegroups.R3(),
            pin.liegroups.SO3(),
            pin.liegroups.SO2(),
            pin.liegroups.SE3(),
            pin.liegroups.SE2(),
            pin.liegroups.R3() * pin.liegroups.SO3(),
        ]:
            q0 = lg.random()
            q1 = lg.random()

            for arg in [pin.ARG0, pin.ARG1]:
                J = lg.dDifference(q0, q1, arg)

                SELF = 0
                J0 = np.random.rand(lg.nv, lg.nv)

                J1 = lg.dDifference(q0, q1, arg, SELF, J0)
                self.assertTrue(np.allclose(np.dot(J, J0), J1))

                J1 = lg.dDifference(q0, q1, arg, J0, SELF)
                self.assertTrue(np.allclose(np.dot(J0, J), J1))

    def test_dIntegrateTransport(self):
        for lg in [
            pin.liegroups.R3(),
            pin.liegroups.SO3(),
            pin.liegroups.SO2(),
            pin.liegroups.SE3(),
            pin.liegroups.SE2(),
            pin.liegroups.R3() * pin.liegroups.SO3(),
        ]:
            q = lg.random()
            v = np.random.rand(lg.nv)

            for arg in [pin.ARG0, pin.ARG1]:
                Jint = lg.dIntegrate(q, v, arg)
                J0 = np.random.rand(lg.nv, lg.nv)
                Jout1 = lg.dIntegrateTransport(q, v, J0, arg)
                Jout1_ref = Jint.dot(J0)
                self.assertApprox(Jout1, Jout1_ref)

    def test_dIntegrateTransport_inverse(self):
        for lg in [
            pin.liegroups.R3(),
            pin.liegroups.SO3(),
            pin.liegroups.SO2(),
            pin.liegroups.SE3(),
            pin.liegroups.SE2(),
            pin.liegroups.R3() * pin.liegroups.SO3(),
        ]:
            q0 = lg.random()
            v = np.random.rand(lg.nv)
            q1 = lg.integrate(q0, v)

            # transport random tangent vector from q1 to q0
            tvec_at_q1 = np.random.rand(lg.nv)
            tvec_at_q0 = lg.dIntegrateTransport(q0, v, tvec_at_q1, pin.ARG0)

            # test reverse direction
            v_r = -v.copy()  # reverse path
            q0_r = lg.integrate(q1, v_r)

            self.assertApprox(q0, q0_r)  # recover init point on manifold

            tvec_at_q1_r = lg.dIntegrateTransport(q1, v_r, tvec_at_q0, pin.ARG0)

            self.assertApprox(tvec_at_q1, tvec_at_q1_r)

            # same test for matrix
            J_at_q1 = np.random.rand(lg.nv, lg.nv)
            J_at_q0 = lg.dIntegrateTransport(q0, v, J_at_q1, pin.ARG0)
            self.assertApprox(
                J_at_q1, lg.dIntegrateTransport(q1, v_r, J_at_q0, pin.ARG0)
            )


if __name__ == "__main__":
    unittest.main()
