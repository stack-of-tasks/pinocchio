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
        self.tau = np.random.rand(self.model.nv)

        self.fext = []
        for _ in range(self.model.njoints):
            self.fext.append(pin.Force.Random())

    def test_aba_derivatives(self):
        res = pin.computeABADerivatives(self.model, self.data, self.q, self.v, self.tau)

        self.assertTrue(len(res) == 3)

        data2 = self.model.createData()
        pin.aba(self.model, data2, self.q, self.v, self.tau)

        self.assertApprox(self.data.ddq, data2.ddq)

        # With external forces
        res = pin.computeABADerivatives(
            self.model, self.data, self.q, self.v, self.tau, self.fext
        )

        self.assertTrue(len(res) == 3)

        pin.aba(self.model, data2, self.q, self.v, self.tau, self.fext)

        self.assertApprox(self.data.ddq, data2.ddq)

    def assertTensorApprox(self, a, b, prec):
        self.assertEqual(a.shape, b.shape)
        self.assertLessEqual(
            np.linalg.norm(a - b), prec * min(np.linalg.norm(a), np.linalg.norm(b))
        )

    def test_aba_second_order_derivatives(self):
        model = self.model
        nv = model.nv
        res = pin.computeABASecondOrderDerivatives(
            model, self.data, self.q, self.v, self.tau
        )

        self.assertTrue(len(res) == 4)
        d2ddq_dqdq, d2ddq_dvdv, d2ddq_dqdv, d2ddq_dtaudq = res
        for tensor in res:
            self.assertEqual(tensor.shape, (nv, nv, nv))

        self.assertTrue(np.array_equal(d2ddq_dqdq, self.data.d2ddq_dqdq))
        self.assertTrue(np.array_equal(d2ddq_dvdv, self.data.d2ddq_dvdv))
        self.assertTrue(np.array_equal(d2ddq_dqdv, self.data.d2ddq_dqdv))
        self.assertTrue(np.array_equal(d2ddq_dtaudq, self.data.d2ddq_dtaudq))

        # Finite differences of the first-order derivatives, with the documented storage
        # order: d2ddq_dxdy[i, j, k] = d/dy_k (d ddq_i / dx_j)
        data_fd = model.createData()
        ddq_dq, ddq_dv, ddq_dtau = (
            np.copy(m)
            for m in pin.computeABADerivatives(model, data_fd, self.q, self.v, self.tau)
        )

        alpha = 1e-7
        d2ddq_dqdq_fd = np.zeros((nv, nv, nv))
        d2ddq_dvdv_fd = np.zeros((nv, nv, nv))
        d2ddq_dqdv_fd = np.zeros((nv, nv, nv))
        d2ddq_dtaudq_fd = np.zeros((nv, nv, nv))
        for k in range(nv):
            v_eps = np.zeros(nv)
            v_eps[k] = alpha

            q_plus = pin.integrate(model, self.q, v_eps)
            ddq_dq_plus, _, ddq_dtau_plus = pin.computeABADerivatives(
                model, data_fd, q_plus, self.v, self.tau
            )
            d2ddq_dqdq_fd[:, :, k] = (ddq_dq_plus - ddq_dq) / alpha
            d2ddq_dtaudq_fd[:, :, k] = (ddq_dtau_plus - ddq_dtau) / alpha

            ddq_dq_plus, ddq_dv_plus, _ = pin.computeABADerivatives(
                model, data_fd, self.q, self.v + v_eps, self.tau
            )
            d2ddq_dvdv_fd[:, :, k] = (ddq_dv_plus - ddq_dv) / alpha
            d2ddq_dqdv_fd[:, :, k] = (ddq_dq_plus - ddq_dq) / alpha

        prec = np.sqrt(alpha)
        self.assertTensorApprox(d2ddq_dqdq, d2ddq_dqdq_fd, prec)
        self.assertTensorApprox(d2ddq_dvdv, d2ddq_dvdv_fd, prec)
        self.assertTensorApprox(d2ddq_dqdv, d2ddq_dqdv_fd, prec)
        self.assertTensorApprox(d2ddq_dtaudq, d2ddq_dtaudq_fd, prec)

        # The tensors used by the algorithm are not allocated with NO_TENSORS
        data_no_tensors = model.createData(pin.DataAllocationOption.NO_TENSORS)
        with self.assertRaises(Exception):
            pin.computeABASecondOrderDerivatives(
                model, data_no_tensors, self.q, self.v, self.tau
            )


if __name__ == "__main__":
    unittest.main()
