import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

class TestInertiaBindings(unittest.TestCase):

    v3zero = zero(3)
    v6zero = zero(6)
    v3ones = ones(3)
    m3zero = zero([3,3])
    m6zero = zero([6,6])
    m3ones = eye(3)
    m4ones = eye(4)

    def test_zero_getters(self):
        Y = se3.Inertia.Zero()
        self.assertTrue(Y.mass == 0)
        self.assertTrue(np.allclose(self.v3zero, Y.lever))
        self.assertTrue(np.allclose(self.m3zero, Y.inertia))

    def test_identity_getters(self):
        Y = se3.Inertia.Identity()
        self.assertTrue(Y.mass == 1)
        self.assertTrue(np.allclose(self.v3zero, Y.lever))
        self.assertTrue(np.allclose(self.m3ones, Y.inertia))

    # TODO: this is not nice, since a random matrix can *in theory* be unity
    def test_setRandom(self):
        Y = se3.Inertia.Identity()
        Y.setRandom()
        self.assertFalse(Y.mass == 1)
        self.assertFalse(np.allclose(self.v3zero, Y.lever))
        self.assertFalse(np.allclose(self.m3ones, Y.inertia))

    def test_setZero(self):
        Y = se3.Inertia.Zero()
        Y.setRandom()
        Y.setZero()
        self.assertTrue(Y.mass == 0)
        self.assertTrue(np.allclose(self.v3zero, Y.lever))
        self.assertTrue(np.allclose(self.m3zero, Y.inertia))

    def test_setIdentity(self):
        Y = se3.Inertia.Zero()
        Y.setIdentity()
        self.assertTrue(Y.mass == 1)
        self.assertTrue(np.allclose(self.v3zero, Y.lever))
        self.assertTrue(np.allclose(self.m3ones, Y.inertia))

    def test_set_mass(self):
        Y = se3.Inertia.Zero()
        Y.mass = 10
        self.assertTrue(np.allclose(Y.mass, 10))

    def test_set_lever(self):
        Y = se3.Inertia.Zero()
        lev = rand(3)
        Y.lever = lev
        self.assertTrue(np.allclose(Y.lever, lev))

    def test_set_inertia(self):
        Y = se3.Inertia.Zero()
        iner = rand([3,3])
        iner = (iner + iner.T) / 2.  # Symmetrize the matrix
        Y.inertia = iner
        self.assertTrue(np.allclose(Y.inertia, iner))

    def test_internal_sums(self):
        Y1 = se3.Inertia.Random()
        Y2 = se3.Inertia.Random()
        Y = Y1 + Y2
        self.assertTrue(np.allclose(Y1.matrix() + Y2.matrix(), Y.matrix()))

    def test_se3_action(self):
        m = se3.SE3.Random()
        Y = se3.Inertia.Random()
        v = se3.Motion.Random()
        self.assertTrue(np.allclose((Y * v).vector, Y.matrix() * v.vector))
        self.assertTrue(np.allclose((m * Y).matrix(),  m.inverse().action.T * Y.matrix() * m.inverse().action))
        self.assertTrue(np.allclose(m.act(Y).matrix(), m.inverse().action.T * Y.matrix() * m.inverse().action))
        self.assertTrue(np.allclose((m.actInv(Y)).matrix(), m.action.T * Y.matrix() * m.action))

if __name__ == '__main__':
    unittest.main()
