import unittest
import pinocchio as pin
from pinocchio.utils import np, npl, rand, zero

from test_case import PinocchioTestCase as TestCase

# This whole file seems to be outdated and superseded by more recent tests
# Probably it should be removed and its contents moved or split somewhere else

class TestSE3(TestCase):
    def setUp(self):
        self.R = rand([3, 3])
        self.R, _, _ = npl.svd(self.R)
        self.p = rand(3)
        self.m = pin.SE3(self.R, self.p)

    def test_se3(self):
        R, p, m = self.R, self.p, self.m
        X = np.vstack([np.hstack([R, pin.skew(p).dot(R)]), np.hstack([zero([3, 3]), R])])
        self.assertApprox(m.action, X)
        M = np.vstack([np.hstack([R, np.expand_dims(p,1)]), np.array([[0., 0., 0., 1.]])])
        self.assertApprox(m.homogeneous, M)
        m2 = pin.SE3.Random()
        self.assertApprox((m * m2).homogeneous, m.homogeneous.dot(m2.homogeneous))
        self.assertApprox((~m).homogeneous, npl.inv(m.homogeneous))

        p = rand(3)
        self.assertApprox(m * p, m.rotation.dot(p) + m.translation)
        self.assertApprox(m.actInv(p), m.rotation.T.dot(p) - m.rotation.T.dot(m.translation))

        # Currently, the different cases do not throw the same exception type.
        # To have a more robust test, only Exception is checked.
        # In the comments, the most specific actual exception class at the time of writing
        p = rand(5)
        with self.assertRaises(Exception): # RuntimeError
            m * p
        with self.assertRaises(Exception): # RuntimeError
            m.actInv(p)
        with self.assertRaises(Exception): # Boost.Python.ArgumentError (subclass of TypeError)
            m.actInv('42')

    def test_motion(self):
        m = self.m
        self.assertApprox(pin.Motion.Zero().vector, zero(6))
        v = pin.Motion.Random()
        self.assertApprox((m * v).vector, m.action.dot(v.vector))
        self.assertApprox((m.actInv(v)).vector, npl.inv(m.action).dot(v.vector))
        vv = v.linear
        vw = v.angular
        self.assertApprox(v.vector, np.concatenate([vv, vw]))
        self.assertApprox((v ^ v).vector, zero(6))

    def test_force(self):
        m = self.m
        self.assertApprox(pin.Force.Zero().vector, zero(6))
        f = pin.Force.Random()
        ff = f.linear
        ft = f.angular
        self.assertApprox(f.vector, np.concatenate([ff, ft]))

        self.assertApprox((m * f).vector, npl.inv(m.action.T).dot(f.vector))
        self.assertApprox((m.actInv(f)).vector, m.action.T.dot(f.vector))
        v = pin.Motion.Random()
        f = pin.Force(np.concatenate([v.vector[3:], v.vector[:3]]))
        self.assertApprox((v ^ f).vector, zero(6))

    def test_inertia(self):
        m = self.m
        Y1 = pin.Inertia.Random()
        Y2 = pin.Inertia.Random()
        Y = Y1 + Y2
        self.assertApprox(Y1.matrix() + Y2.matrix(), Y.matrix())
        v = pin.Motion.Random()
        self.assertApprox((Y * v).vector, Y.matrix().dot(v.vector))
        self.assertApprox((m * Y).matrix(), m.inverse().action.T.dot(Y.matrix()).dot(m.inverse().action))
        self.assertApprox((m.actInv(Y)).matrix(), m.action.T.dot(Y.matrix()).dot(m.action))

    def test_cross(self):
        m = pin.Motion.Random()
        f = pin.Force.Random()
        self.assertApprox(m ^ m, m.cross(m))
        self.assertApprox(m ^ f, m.cross(f))
        with self.assertRaises(TypeError):
            m ^ 2

if __name__ == '__main__':
    unittest.main()
