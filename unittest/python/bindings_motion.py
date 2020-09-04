import unittest
import pinocchio as pin
import numpy as np
from pinocchio.utils import zero,rand

class TestMotionBindings(unittest.TestCase):

    def test_zero_getters(self):
        v = pin.Motion.Zero()
        self.assertTrue(np.allclose(zero(3), v.linear))
        self.assertTrue(np.allclose(zero(3), v.angular))
        self.assertTrue(np.allclose(zero(6), v.vector))

    # TODO: this is not nice, since a random vector can *in theory* be zero
    def test_setRandom(self):
        v = pin.Motion.Zero()
        v.setRandom()
        self.assertFalse(np.allclose(zero(3), v.linear))
        self.assertFalse(np.allclose(zero(3), v.angular))
        self.assertFalse(np.allclose(zero(6), v.vector))

    def test_setZero(self):
        v = pin.Motion.Zero()
        v.setRandom()
        v.setZero()
        self.assertTrue(np.allclose(zero(3), v.linear))
        self.assertTrue(np.allclose(zero(3), v.angular))
        self.assertTrue(np.allclose(zero(6), v.vector))

    def test_set_linear(self):
        v = pin.Motion.Zero()
        lin =  rand(3)
        v.linear = lin
        self.assertTrue(np.allclose(v.linear, lin))

        v.linear[1] = 1.
        self.assertTrue(v.linear[1] == 1.)

    def test_set_angular(self):
        v = pin.Motion.Zero()
        ang = rand(3)
        v.angular = ang
        self.assertTrue(np.allclose(v.angular, ang))

        v.angular[1] = 1.
        self.assertTrue(v.angular[1] == 1.)

    def test_set_vector(self):
        v = pin.Motion.Zero()
        vec = rand(6)
        v.vector = vec
        self.assertTrue(np.allclose(v.vector, vec))

    def test_internal_sums(self):
        v1 = pin.Motion.Random()
        v2 = pin.Motion.Random()
        self.assertTrue(np.allclose((v1+v2).vector,v1.vector + v2.vector))
        self.assertTrue(np.allclose((v1 - v2).vector, v1.vector - v2.vector))

    def test_se3_action(self):
        m = pin.SE3.Random()
        v = pin.Motion.Random()
        self.assertTrue(np.allclose((m * v).vector,  m.action.dot(v.vector)))
        self.assertTrue(np.allclose(m.act(v).vector, m.action.dot(v.vector)))
        self.assertTrue(np.allclose((m.actInv(v)).vector, np.linalg.inv(m.action).dot(v.vector)))
        self.assertTrue(np.allclose((v ^ v).vector, zero(6)))

    def test_conversion(self):

        m = pin.Motion.Random()
        m_array = np.array(m)

        m_from_array = pin.Motion(m_array)

        self.assertTrue(m_from_array == m)

if __name__ == '__main__':
    unittest.main()
