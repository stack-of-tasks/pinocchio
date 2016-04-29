import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

class TestMotionBindings(unittest.TestCase):

    v3zero = zero(3)
    v6zero = zero(6)
    v3ones = ones(3)
    m3zero = zero([3,3])
    m3ones = eye(3)
    m4ones = eye(4)

    def test_zero_getters(self):
        v = se3.Motion.Zero()
        self.assertTrue(np.allclose(self.v3zero, v.linear))
        self.assertTrue(np.allclose(self.v3zero, v.angular))
        self.assertTrue(np.allclose(self.v6zero, v.vector))

    def test_setRandom(self):
        v = se3.Motion.Zero()
        v.setRandom()
        self.assertFalse(np.allclose(self.v3zero, v.linear))
        self.assertFalse(np.allclose(self.v3zero, v.angular))
        self.assertFalse(np.allclose(self.v6zero, v.vector))

    def test_setZero(self):
        v = se3.Motion.Zero()
        v.setRandom()
        v.setZero()
        self.assertTrue(np.allclose(self.v3zero, v.linear))
        self.assertTrue(np.allclose(self.v3zero, v.angular))
        self.assertTrue(np.allclose(self.v6zero, v.vector))

    def test_set_linear(self):
        v = se3.Motion.Zero()
        lin =  rand(3)  # TODO np.matrix([1,2,3],np.double) OR np.matrix( np.array([1,2,3], np.double), np.double)
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
        self.assertTrue(np.allclose((m * v).vector, m.action * v.vector))
        self.assertTrue(np.allclose((m.actInv(v)).vector, np.linalg.inv(m.action) * v.vector))
        self.assertTrue(np.allclose((v ** v).vector, zero(6)))


