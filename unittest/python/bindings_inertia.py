import unittest
import pinocchio as pin
import numpy as np
from pinocchio.utils import eye,zero,rand

from test_case import PinocchioTestCase as TestCase

class TestInertiaBindings(TestCase):

    def test_zero_getters(self):
        Y = pin.Inertia.Zero()
        self.assertTrue(Y.mass == 0)
        self.assertTrue(np.allclose(zero(3), Y.lever))
        self.assertTrue(np.allclose(zero([3,3]), Y.inertia))

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
        self.assertTrue(np.allclose(zero([3,3]), Y.inertia))

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
        iner = rand([3,3])
        iner = (iner + iner.T) / 2.  # Symmetrize the matrix
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
        self.assertTrue(np.allclose((m * Y).matrix(),  m.inverse().action.T.dot(Y.matrix()).dot(m.inverse().action)))
        self.assertTrue(np.allclose(m.act(Y).matrix(), m.inverse().action.T.dot(Y.matrix()).dot(m.inverse().action)))
        self.assertTrue(np.allclose((m.actInv(Y)).matrix(), m.action.T.dot(Y.matrix()).dot(m.action)))

    def test_dynamic_parameters(self):
        I = pin.Inertia.Random()

        v = I.toDynamicParameters()

        self.assertApprox(v[0], I.mass)
        self.assertApprox(v[1:4], I.mass * I.lever)

        I_o = I.inertia + I.mass * pin.skew(I.lever).transpose().dot(pin.skew(I.lever))
        I_ov = np.array(
                [[float(v[4]), float(v[5]), float(v[7])],
                 [float(v[5]), float(v[6]), float(v[8])],
                 [float(v[7]), float(v[8]), float(v[9])]
                ]
               )

        self.assertApprox(I_o, I_ov)

        I2 = pin.Inertia.FromDynamicParameters(v)
        self.assertApprox(I2, I)
    
    def test_array(self):
        I = pin.Inertia.Random()
        I_array = np.array(I)

        self.assertApprox(I_array,I.matrix())

if __name__ == '__main__':
    unittest.main()
