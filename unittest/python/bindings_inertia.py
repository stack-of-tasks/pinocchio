import unittest

import numpy as np
import pinocchio as pin
from pinocchio.utils import eye, rand, zero
from test_case import PinocchioTestCase as TestCase


class TestInertiaBindings(TestCase):
    def test_zero_getters(self):
        Y = pin.Inertia.Zero()
        self.assertTrue(Y.mass == 0)
        self.assertTrue(np.allclose(zero(3), Y.lever))
        self.assertTrue(np.allclose(zero([3, 3]), Y.inertia))

    def test_identity_getters(self):
        Y = pin.Inertia.Identity()
        self.assertTrue(Y.mass == 1)
        self.assertTrue(np.allclose(zero(3), Y.lever))
        self.assertTrue(np.allclose(eye(3), Y.inertia))

    # TODO: this is not nice, since a random matrix can *in theory* be unity
    def test_setRandom(self):
        Y = pin.Inertia.Identity()
        Y.setRandom()
        self.assertFalse(Y.mass == 1)
        self.assertFalse(np.allclose(zero(3), Y.lever))
        self.assertFalse(np.allclose(eye(3), Y.inertia))

    def test_setZero(self):
        Y = pin.Inertia.Zero()
        Y.setRandom()
        Y.setZero()
        self.assertTrue(Y.mass == 0)
        self.assertTrue(np.allclose(zero(3), Y.lever))
        self.assertTrue(np.allclose(zero([3, 3]), Y.inertia))

    def test_setIdentity(self):
        Y = pin.Inertia.Zero()
        Y.setIdentity()
        self.assertTrue(Y.mass == 1)
        self.assertTrue(np.allclose(zero(3), Y.lever))
        self.assertTrue(np.allclose(eye(3), Y.inertia))

    def test_set_mass(self):
        Y = pin.Inertia.Zero()
        Y.mass = 10
        self.assertTrue(np.allclose(Y.mass, 10))

    def test_set_lever(self):
        Y = pin.Inertia.Zero()
        lev = rand(3)
        Y.lever = lev
        self.assertTrue(np.allclose(Y.lever, lev))

    def test_set_inertia(self):
        Y = pin.Inertia.Zero()
        iner = rand([3, 3])
        iner = (iner + iner.T) / 2.0  # Symmetrize the matrix
        Y.inertia = iner
        self.assertTrue(np.allclose(Y.inertia, iner))

    def test_internal_sums(self):
        Y1 = pin.Inertia.Random()
        Y2 = pin.Inertia.Random()
        Y = Y1 + Y2
        self.assertTrue(np.allclose(Y1.matrix() + Y2.matrix(), Y.matrix()))

    def test_se3_action(self):
        m = pin.SE3.Random()
        Y = pin.Inertia.Random()
        v = pin.Motion.Random()
        self.assertTrue(np.allclose((Y * v).vector, Y.matrix().dot(v.vector)))
        self.assertTrue(
            np.allclose(
                (m * Y).matrix(),
                m.inverse().action.T.dot(Y.matrix()).dot(m.inverse().action),
            )
        )
        self.assertTrue(
            np.allclose(
                m.act(Y).matrix(),
                m.inverse().action.T.dot(Y.matrix()).dot(m.inverse().action),
            )
        )
        self.assertTrue(
            np.allclose(
                (m.actInv(Y)).matrix(), m.action.T.dot(Y.matrix()).dot(m.action)
            )
        )

    def test_dynamic_parameters(self):
        In = pin.Inertia.Random()

        v = In.toDynamicParameters()

        self.assertApprox(v[0], In.mass)
        self.assertApprox(v[1:4], In.mass * In.lever)

        I_o = In.inertia + In.mass * pin.skew(In.lever).transpose().dot(
            pin.skew(In.lever)
        )
        I_ov = np.array(
            [
                [float(v[4]), float(v[5]), float(v[7])],
                [float(v[5]), float(v[6]), float(v[8])],
                [float(v[7]), float(v[8]), float(v[9])],
            ]
        )

        self.assertApprox(I_o, I_ov)

        I2 = pin.Inertia.FromDynamicParameters(v)
        self.assertApprox(I2, In)

    def test_pseudo_inertia(self):
        I = pin.Inertia.Random()
        pseudo = I.toPseudoInertia()
        self.assertTrue(np.all(np.linalg.eigvals(pseudo) > 0))
        I_back = pin.Inertia.FromPseudoInertia(pseudo)
        self.assertApprox(I_back.mass, I.mass)
        self.assertApprox(I_back.lever, I.lever)
        self.assertApprox(I_back.inertia, I.inertia)

    def test_log_cholesky(self):
        eta = np.random.randn(10)
        alpha, d1, d2, d3, s12, s23, s13, t1, t2, t3 = eta
        # Compute the exponential terms
        exp_alpha = np.exp(alpha)
        exp_d1 = np.exp(d1)
        exp_d2 = np.exp(d2)
        exp_d3 = np.exp(d3)

        # Create the matrix U
        U = exp_alpha * np.array(
            [
                [exp_d1, s12, s13, t1],
                [0, exp_d2, s23, t2],
                [0, 0, exp_d3, t3],
                [0, 0, 0, 1],
            ]
        )
        pseudo_chol = U @ U.T

        inertia = pin.Inertia.FromLogCholeskyParameters(eta)
        pseudo = inertia.toPseudoInertia()
        self.assertTrue(np.all(np.linalg.eigvals(pseudo) > 0))
        self.assertApprox(pseudo, pseudo_chol)

    def test_array(self):
        In = pin.Inertia.Random()
        I_array = np.array(In)

        self.assertApprox(I_array, In.matrix())

    def test_several_init(self):
        for _ in range(100000):
            In = pin.Inertia.Random() + pin.Inertia.Random()
            s = In.__str__()
            self.assertTrue(s != "")


if __name__ == "__main__":
    unittest.main()
