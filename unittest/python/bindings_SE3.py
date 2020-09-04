import unittest
import pinocchio as pin
import numpy as np
from pinocchio.utils import eye,zero,rand

ones = lambda n: np.ones([n, 1] if isinstance(n, int) else n)

class TestSE3Bindings(unittest.TestCase):

    def test_identity(self):
        transform = pin.SE3.Identity()
        self.assertTrue(np.allclose(zero(3),transform.translation))
        self.assertTrue(np.allclose(eye(3), transform.rotation))
        self.assertTrue(np.allclose(eye(4), transform.homogeneous))
        self.assertTrue(np.allclose(eye(6), transform.action))
        transform.setRandom()
        transform.setIdentity()
        self.assertTrue(np.allclose(eye(4), transform.homogeneous))

    def test_constructor(self):
        M = pin.SE3.Random()
        quat = pin.Quaternion(M.rotation)
        M_from_quat = pin.SE3(quat,M.translation)
        self.assertTrue(M_from_quat.isApprox(M))

    def test_get_translation(self):
        transform = pin.SE3.Identity()
        self.assertTrue(np.allclose(transform.translation, zero(3)))

    def test_get_rotation(self):
        transform = pin.SE3.Identity()
        self.assertTrue(np.allclose(transform.rotation, eye(3)))

    def test_set_translation(self):
        transform = pin.SE3.Identity()
        transform.translation = ones(3)
        self.assertFalse(np.allclose(transform.translation, zero(3)))
        self.assertTrue(np.allclose(transform.translation, ones(3)))

    def test_set_rotation(self):
        transform = pin.SE3.Identity()
        transform.rotation = zero([3,3])
        self.assertFalse(np.allclose(transform.rotation, eye(3)))
        self.assertTrue(np.allclose(transform.rotation, zero([3,3])))

    def test_homogeneous(self):
        amb = pin.SE3.Random()
        H = amb.homogeneous
        self.assertTrue(np.allclose(H[0:3,0:3], amb.rotation))  # top left 33 corner = rotation of amb
        self.assertTrue(np.allclose(H[0:3,3], amb.translation)) # top 3 of last column = translation of amb
        self.assertTrue(np.allclose(H[3,:], [0.,0.,0.,1.]))     # last row = 0 0 0 1

        amb_from_H = pin.SE3(H)
        self.assertTrue(amb_from_H.isApprox(amb))

    def test_action_matrix(self):
        amb = pin.SE3.Random()        
        aXb = amb.action
        self.assertTrue(np.allclose(aXb[:3,:3], amb.rotation)) # top left 33 corner = rotation of amb
        self.assertTrue(np.allclose(aXb[3:,3:], amb.rotation)) # bottom right 33 corner = rotation of amb
        tblock = pin.skew(amb.translation).dot(amb.rotation)
        self.assertTrue(np.allclose(aXb[:3,3:], tblock))       # top right 33 corner = translation + rotation
        self.assertTrue(np.allclose(aXb[3:,:3], zero([3,3])))  # bottom left 33 corner = 0

    def test_inverse(self):
        amb = pin.SE3.Random()
        aMb = amb.homogeneous
        bma = amb.inverse()
        self.assertTrue(np.allclose(np.linalg.inv(aMb), bma.homogeneous))

    def test_internal_product_vs_homogeneous(self):
        amb = pin.SE3.Random()
        bmc = pin.SE3.Random()
        amc = amb*bmc
        cma = amc.inverse()

        aMb = amb.homogeneous
        bMc = bmc.homogeneous
        aMc = amc.homogeneous
        cMa = np.linalg.inv(aMc)

        self.assertTrue(np.allclose(aMb.dot(bMc), aMc))
        self.assertTrue(np.allclose(cma.homogeneous,cMa))

    def test_internal_product_vs_action(self):
        amb = pin.SE3.Random()
        bmc = pin.SE3.Random()
        amc = amb * bmc
        cma = amc.inverse()
        
        aXb = amb.action
        bXc = bmc.action
        aXc = amc.action
        cXa = np.linalg.inv(aXc)

        self.assertTrue(np.allclose(aXb.dot(bXc), aXc))
        self.assertTrue(np.allclose(cma.action,cXa))

    def test_point_action(self):
        amb = pin.SE3.Random()
        aMb = amb.homogeneous
        p_homogeneous = rand(4)
        p_homogeneous[3] = 1
        p = p_homogeneous[0:3].copy()

        # act
        self.assertTrue(np.allclose(amb.act(p),(aMb.dot(p_homogeneous))[0:3]))

        # actinv
        bMa = np.linalg.inv(aMb)
        bma = amb.inverse()
        self.assertTrue(np.allclose(bma.act(p), (bMa.dot(p_homogeneous))[0:3]))

    def test_member(self):
        M = pin.SE3.Random()
        trans = M.translation
        M.translation[2] = 1.

        self.assertTrue(trans[2] == M.translation[2])

    def test_conversions(self):
        def compute (m):
            tq_vec = pin.SE3ToXYZQUAT      (m)
            tq_tup = pin.SE3ToXYZQUATtuple (m)
            mm_vec = pin.XYZQUATToSE3 (tq_vec)
            mm_tup = pin.XYZQUATToSE3 (tq_tup)
            mm_lis = pin.XYZQUATToSE3 (list(tq_tup))
            return tq_vec, tq_tup, mm_vec, mm_tup, mm_lis

        m = pin.SE3.Identity()
        tq_vec, tq_tup, mm_vec, mm_tup, mm_lis = compute (m)
        self.assertTrue(np.allclose(m.homogeneous, mm_tup.homogeneous))
        self.assertTrue(np.allclose(m.homogeneous, mm_vec.homogeneous))
        self.assertTrue(np.allclose(m.homogeneous, mm_lis.homogeneous))
        for i in range(6):
            self.assertTrue(tq_vec[i] == 0 and tq_tup[i] == 0)
        self.assertTrue(tq_vec[6] == 1 and tq_tup[6] == 1)

        m = pin.SE3.Random()
        tq_vec, tq_tup, mm_vec, mm_tup, mm_lis = compute (m)
        self.assertTrue (len(tq_vec) == 7)
        self.assertTrue (len(tq_tup) == 7)
        for a,b in zip(tq_vec,tq_tup):
            self.assertTrue (a==b)
        self.assertTrue(np.allclose(m.homogeneous, mm_tup.homogeneous))
        self.assertTrue (mm_vec == mm_tup)
        self.assertTrue (mm_vec == mm_lis)

        M = pin.SE3.Random()
        h = np.array(M)

        M_from_h = pin.SE3(h)
        self.assertTrue(M == M_from_h)

if __name__ == '__main__':
    unittest.main()
