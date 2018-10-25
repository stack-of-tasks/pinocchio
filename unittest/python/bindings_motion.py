import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import zero,rand

class TestMotionBindings(unittest.TestCase):

    def test_zero_getters(self):
        v = se3.Motion.Zero()
        self.assertTrue(np.allclose(zero(3), v.linear))
        self.assertTrue(np.allclose(zero(3), v.angular))
        self.assertTrue(np.allclose(zero(6), v.vector))

    # TODO: this is not nice, since a random vector can *in theory* be zero
    def test_setRandom(self):
        v = se3.Motion.Zero()
        v.setRandom()
        self.assertFalse(np.allclose(zero(3), v.linear))
        self.assertFalse(np.allclose(zero(3), v.angular))
        self.assertFalse(np.allclose(zero(6), v.vector))

    def test_setZero(self):
        v = se3.Motion.Zero()
        v.setRandom()
        v.setZero()
        self.assertTrue(np.allclose(zero(3), v.linear))
        self.assertTrue(np.allclose(zero(3), v.angular))
        self.assertTrue(np.allclose(zero(6), v.vector))

    def test_set_linear(self):
        v = se3.Motion.Zero()
        lin =  rand(3)
        v.linear = lin
        self.assertTrue(np.allclose(v.linear, lin))

    def test_set_angular(self):
        v = se3.Motion.Zero()
        ang = rand(3)
        v.angular = ang
        self.assertTrue(np.allclose(v.angular, ang))

    def test_set_vector(self):
        v = se3.Motion.Zero()
        vec = rand(6)
        v.vector = vec
        self.assertTrue(np.allclose(v.vector, vec))

    def test_internal_sums(self):
        v1 = se3.Motion.Random()
        v2 = se3.Motion.Random()
        self.assertTrue(np.allclose((v1+v2).vector,v1.vector + v2.vector))
        self.assertTrue(np.allclose((v1 - v2).vector, v1.vector - v2.vector))

    def test_se3_action(self):
        m = se3.SE3.Random()
        v = se3.Motion.Random()
        self.assertTrue(np.allclose((m * v).vector,  m.action * v.vector))
        self.assertTrue(np.allclose(m.act(v).vector, m.action * v.vector))
        self.assertTrue(np.allclose((m.actInv(v)).vector, np.linalg.inv(m.action) * v.vector))
        self.assertTrue(np.allclose((v ^ v).vector, zero(6)))

if __name__ == '__main__':
    unittest.main()
