import pinocchio as se3
from pinocchio.utils import np, npl, rand, skew, zero

from test_case import TestCase

# This whole file seems to be outdated and superseded by more recent tests
# Probably it should be removed and its contents moved or split somewhere else

class TestSE3(TestCase):
    def setUp(self):
        self.R = rand([3, 3])
        self.R, _, _ = npl.svd(self.R)
        self.p = rand(3)
        self.m = se3.SE3(self.R, self.p)

    def test_se3(self):
        R, p, m = self.R, self.p, self.m
        X = np.vstack([np.hstack([R, skew(p) * R]), np.hstack([zero([3, 3]), R])])
        self.assertApprox(m.action, X)
        M = np.vstack([np.hstack([R, p]), np.matrix([0., 0., 0., 1.], np.double)])
        self.assertApprox(m.homogeneous, M)
        m2 = se3.SE3.Random()
        self.assertApprox((m * m2).homogeneous, m.homogeneous * m2.homogeneous)
        self.assertApprox((~m).homogeneous, npl.inv(m.homogeneous))

        p = rand(3)
        self.assertApprox(m * p, m.rotation * p + m.translation)
        self.assertApprox(m.actInv(p), m.rotation.T * p - m.rotation.T * m.translation)

        ## not supported
        # p = np.vstack([p, 1])
        # self.assertApprox(m * p, m.homogeneous * p)
        # self.assertApprox(m.actInv(p), npl.inv(m.homogeneous) * p)

        ## not supported
        # p = rand(6)
        # self.assertApprox(m * p, m.action * p)
        # self.assertApprox(m.actInv(p), npl.inv(m.action) * p)

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
        self.assertApprox(se3.Motion.Zero().vector, zero(6))
        v = se3.Motion.Random()
        self.assertApprox((m * v).vector, m.action * v.vector)
        self.assertApprox((m.actInv(v)).vector, npl.inv(m.action) * v.vector)
        vv = v.linear
        vw = v.angular
        self.assertApprox(v.vector, np.vstack([vv, vw]))
        self.assertApprox((v ^ v).vector, zero(6))

    def test_force(self):
        m = self.m
        self.assertApprox(se3.Force.Zero().vector, zero(6))
        f = se3.Force.Random()
        ff = f.linear
        ft = f.angular
        self.assertApprox(f.vector, np.vstack([ff, ft]))

        self.assertApprox((m * f).vector, npl.inv(m.action.T) * f.vector)
        self.assertApprox((m.actInv(f)).vector, m.action.T * f.vector)
        v = se3.Motion.Random()
        f = se3.Force(np.vstack([v.vector[3:], v.vector[:3]]))
        self.assertApprox((v ^ f).vector, zero(6))

    def test_inertia(self):
        m = self.m
        Y1 = se3.Inertia.Random()
        Y2 = se3.Inertia.Random()
        Y = Y1 + Y2
        self.assertApprox(Y1.matrix() + Y2.matrix(), Y.matrix())
        v = se3.Motion.Random()
        self.assertApprox((Y * v).vector, Y.matrix() * v.vector)
        self.assertApprox((m * Y).matrix(), m.inverse().action.T * Y.matrix() * m.inverse().action)
        self.assertApprox((m.actInv(Y)).matrix(), m.action.T * Y.matrix() * m.action)

    def test_cross(self):
        m = se3.Motion.Random()
        f = se3.Force.Random()
        self.assertApprox(m ^ m, m.cross(m))
        self.assertApprox(m ^ f, m.cross(f))
        with self.assertRaises(TypeError):
            m ^ 2

    def test_exp(self):
        m = se3.Motion.Random()
        self.assertApprox(se3.exp(m), se3.exp6FromMotion(m))

if __name__ == '__main__':
    unittest.main()