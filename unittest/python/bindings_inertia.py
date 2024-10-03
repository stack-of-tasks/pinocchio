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

        # Test FromDynamicParameters raise an exception if the wrong
        # vector size is provided
        with self.assertRaises(ValueError):
            pin.Inertia.FromDynamicParameters(np.array([]))

    def test_pseudo_inertia(self):
        In = pin.Inertia.Random()

        pseudo = In.toPseudoInertia()

        # test accessing mass, h, sigma
        self.assertApprox(pseudo.mass, In.mass)
        self.assertApprox(pseudo.h, In.mass * In.lever)
        self.assertEqual(pseudo.sigma.shape, (3, 3))

        # test toMatrix
        _ = pseudo.toMatrix()

        # test toDynamicParameters
        params = pseudo.toDynamicParameters()

        # test fromDynamicParameters
        pseudo2 = pin.PseudoInertia.FromDynamicParameters(params)
        self.assertApprox(pseudo.mass, pseudo2.mass)
        self.assertApprox(pseudo.h, pseudo2.h)
        self.assertApprox(pseudo.sigma, pseudo2.sigma)

        # Test FromDynamicParameters raise an exception if the wrong
        # vector size is provided
        with self.assertRaises(ValueError):
            pin.PseudoInertia.FromDynamicParameters(np.array([]))

        # test fromMatrix
        pseudo3 = pin.PseudoInertia.FromMatrix(pseudo.toMatrix())
        self.assertApprox(pseudo.mass, pseudo3.mass)
        self.assertApprox(pseudo.h, pseudo3.h)
        self.assertApprox(pseudo.sigma, pseudo3.sigma)

        # test fromInertia
        pseudo4 = pin.PseudoInertia.FromInertia(In)
        self.assertApprox(pseudo.mass, pseudo4.mass)
        self.assertApprox(pseudo.h, pseudo4.h)
        self.assertApprox(pseudo.sigma, pseudo4.sigma)

        # test toInertia
        self.assertApprox(pseudo4.toInertia(), In)

        # test from PseudoInertia
        pin.PseudoInertia(pseudo)

        # test array
        pseudo_array = np.array(pseudo)
        self.assertApprox(pseudo_array, pseudo.toMatrix())

    def test_log_cholesky(self):
        log_cholesky = pin.LogCholeskyParameters(np.random.randn(10))
        In = pin.Inertia.FromLogCholeskyParameters(log_cholesky)

        # Test constructor with wrong vector size
        with self.assertRaises(ValueError):
            log_cholesky = pin.LogCholeskyParameters(np.array([]))

        # test accessing parameters
        self.assertEqual(log_cholesky.parameters.shape, (10,))

        # test toDynamicParameters
        params = log_cholesky.toDynamicParameters()
        params2 = In.toDynamicParameters()
        self.assertApprox(params, params2)

        # test toPseudoInertia
        pseudo = log_cholesky.toPseudoInertia()
        pseudo2 = In.toPseudoInertia()
        self.assertApprox(pseudo.mass, pseudo2.mass)
        self.assertApprox(pseudo.h, pseudo2.h)
        self.assertApprox(pseudo.sigma, pseudo2.sigma)

        # test toInertia
        In2 = log_cholesky.toInertia()
        self.assertApprox(In, In2)

        # test calculateJacobian
        log_cholesky.calculateJacobian()

        # test array
        log_cholesky_array = np.array(log_cholesky)
        self.assertApprox(log_cholesky_array, log_cholesky.parameters)

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
