import unittest
import pinocchio as pin
import numpy as np
from pinocchio.utils import eye,zero,rand

class TestInertiaBindings(unittest.TestCase):

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
        self.assertTrue(np.allclose((Y * v).vector, Y.matrix() * v.vector))
        self.assertTrue(np.allclose((m * Y).matrix(),  m.inverse().action.T * Y.matrix() * m.inverse().action))
        self.assertTrue(np.allclose(m.act(Y).matrix(), m.inverse().action.T * Y.matrix() * m.inverse().action))
        self.assertTrue(np.allclose((m.actInv(Y)).matrix(), m.action.T * Y.matrix() * m.action))

if __name__ == '__main__':
    unittest.main()
