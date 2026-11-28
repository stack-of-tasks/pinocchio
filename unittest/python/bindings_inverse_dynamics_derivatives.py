import unittest

import numpy as np
import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


class TestDeriavtives(TestCase):
    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

        qmax = np.full((self.model.nq, 1), np.pi)
        self.q = pin.randomConfiguration(self.model, -qmax, qmax)
        self.v = np.random.rand(self.model.nv)
        self.a = np.random.rand(self.model.nv)

        self.fext = []
        for _ in range(self.model.njoints):
            self.fext.append(pin.Force.Random())

    def test_rnea_derivatives(self):
        res = pin.computeRNEADerivatives(self.model, self.data, self.q, self.v, self.a)

        self.assertTrue(len(res) == 3)

        data2 = self.model.createData()
        pin.rnea(self.model, data2, self.q, self.v, self.a)

        self.assertApprox(self.data.ddq, data2.ddq)

        # With external forces
        res = pin.computeRNEADerivatives(
            self.model, self.data, self.q, self.v, self.a, self.fext
        )

        self.assertTrue(len(res) == 3)

        pin.rnea(self.model, data2, self.q, self.v, self.a, self.fext)

        self.assertApprox(self.data.ddq, data2.ddq)

    def assertTensorApprox(self, a, b, prec):
        self.assertEqual(a.shape, b.shape)
        self.assertLessEqual(
            np.linalg.norm(a - b), prec * min(np.linalg.norm(a), np.linalg.norm(b))
        )

    def test_rnea_second_order_derivatives(self):
        model = self.model
        nv = model.nv
        res = pin.computeRNEASecondOrderDerivatives(
            model, self.data, self.q, self.v, self.a
        )

        self.assertTrue(len(res) == 4)
        d2tau_dqdq, d2tau_dvdv, d2tau_dqdv, d2tau_dadq = res
        for tensor in res:
            self.assertEqual(tensor.shape, (nv, nv, nv))

        self.assertTrue(np.array_equal(d2tau_dqdq, self.data.d2tau_dqdq))
        self.assertTrue(np.array_equal(d2tau_dvdv, self.data.d2tau_dvdv))
        self.assertTrue(np.array_equal(d2tau_dqdv, self.data.d2tau_dqdv))
        self.assertTrue(np.array_equal(d2tau_dadq, self.data.d2tau_dadq))

        # Finite differences of the first-order derivatives, with the documented storage
        # order: d2tau_dxdy[i, j, k] = d/dy_k (d tau_i / dx_j)
        data_fd = model.createData()
        dtau_dq, dtau_dv, M = (
            np.copy(m)
            for m in pin.computeRNEADerivatives(model, data_fd, self.q, self.v, self.a)
        )

        alpha = 1e-7
        d2tau_dqdq_fd = np.zeros((nv, nv, nv))
        d2tau_dvdv_fd = np.zeros((nv, nv, nv))
        d2tau_dqdv_fd = np.zeros((nv, nv, nv))
        d2tau_dadq_fd = np.zeros((nv, nv, nv))
        for k in range(nv):
            v_eps = np.zeros(nv)
            v_eps[k] = alpha

            q_plus = pin.integrate(model, self.q, v_eps)
            dtau_dq_plus, _, M_plus = pin.computeRNEADerivatives(
                model, data_fd, q_plus, self.v, self.a
            )
            d2tau_dqdq_fd[:, :, k] = (dtau_dq_plus - dtau_dq) / alpha
            d2tau_dadq_fd[:, :, k] = (M_plus - M) / alpha

            dtau_dq_plus, dtau_dv_plus, _ = pin.computeRNEADerivatives(
                model, data_fd, self.q, self.v + v_eps, self.a
            )
            d2tau_dvdv_fd[:, :, k] = (dtau_dv_plus - dtau_dv) / alpha
            d2tau_dqdv_fd[:, :, k] = (dtau_dq_plus - dtau_dq) / alpha

        prec = np.sqrt(alpha)
        self.assertTensorApprox(d2tau_dqdq, d2tau_dqdq_fd, prec)
        self.assertTensorApprox(d2tau_dvdv, d2tau_dvdv_fd, prec)
        self.assertTensorApprox(d2tau_dqdv, d2tau_dqdv_fd, prec)
        self.assertTensorApprox(d2tau_dadq, d2tau_dadq_fd, prec)

        # The output tensors are not allocated with NO_TENSORS
        data_no_tensors = model.createData(pin.DataAllocationOption.NO_TENSORS)
        with self.assertRaises(Exception):
            pin.computeRNEASecondOrderDerivatives(
                model, data_no_tensors, self.q, self.v, self.a
            )

    def test_generalized_gravity_derivatives(self):
        res = pin.computeGeneralizedGravityDerivatives(self.model, self.data, self.q)

        data2 = self.model.createData()
        ref, _, _ = pin.computeRNEADerivatives(
            self.model, data2, self.q, self.v * 0, self.a * 0
        )

        self.assertApprox(res, ref)

    def test_static_torque_derivatives(self):
        res = pin.computeStaticTorqueDerivatives(
            self.model, self.data, self.q, self.fext
        )

        data2 = self.model.createData()
        ref, _, _ = pin.computeRNEADerivatives(
            self.model, data2, self.q, self.v * 0, self.a * 0, self.fext
        )

        self.assertApprox(res, ref)


if __name__ == "__main__":
    unittest.main()
