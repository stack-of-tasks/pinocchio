import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand,skew

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

class TestSE3Bindings(unittest.TestCase):

    def test_identity(self):
        transform = se3.SE3.Identity()
        self.assertTrue(np.allclose(zero(3),transform.translation))
        self.assertTrue(np.allclose(eye(3), transform.rotation))
        self.assertTrue(np.allclose(eye(4), transform.homogeneous))
        self.assertTrue(np.allclose(eye(6), transform.action))
        transform.setRandom()
        transform.setIdentity()
        self.assertTrue(np.allclose(eye(4), transform.homogeneous))

    def test_get_translation(self):
        transform = se3.SE3.Identity()
        self.assertTrue(np.allclose(transform.translation, zero(3)))

    def test_get_rotation(self):
        transform = se3.SE3.Identity()
        self.assertTrue(np.allclose(transform.rotation, eye(3)))

    def test_set_translation(self):
        transform = se3.SE3.Identity()
        transform.translation = ones(3)
        self.assertFalse(np.allclose(transform.translation, zero(3)))
        self.assertTrue(np.allclose(transform.translation, ones(3)))

    def test_set_rotation(self):
        transform = se3.SE3.Identity()
        transform.rotation = zero([3,3])
        self.assertFalse(np.allclose(transform.rotation, eye(3)))
        self.assertTrue(np.allclose(transform.rotation, zero([3,3])))

    def test_homogeneous(self):
        amb = se3.SE3.Random()
        aMb = amb.homogeneous
        self.assertTrue(np.allclose(aMb[0:3,0:3], amb.rotation))  # top left 33 corner = rotation of amb
        self.assertTrue(np.allclose(aMb[0:3,3], amb.translation)) # top 3 of last column = translation of amb
        self.assertTrue(np.allclose(aMb[3,:], [0.,0.,0.,1.]))     # last row = 0 0 0 1

    def test_action_matrix(self):
        amb = se3.SE3.Random()        
        aXb = amb.action
        self.assertTrue(np.allclose(aXb[:3,:3], amb.rotation)) # top left 33 corner = rotation of amb
        self.assertTrue(np.allclose(aXb[3:,3:], amb.rotation)) # bottom right 33 corner = rotation of amb
        tblock = skew(amb.translation)*amb.rotation
        self.assertTrue(np.allclose(aXb[:3,3:], tblock))       # top right 33 corner = translation + rotation
        self.assertTrue(np.allclose(aXb[3:,:3], zero([3,3])))  # bottom left 33 corner = 0

    def test_inverse(self):
        amb = se3.SE3.Random()
        aMb = amb.homogeneous
        bma = amb.inverse()
        self.assertTrue(np.allclose(np.linalg.inv(aMb), bma.homogeneous))

    def test_internal_product_vs_homogeneous(self):
        amb = se3.SE3.Random()
        bmc = se3.SE3.Random()
        amc = amb*bmc
        cma = amc.inverse()

        aMb = amb.homogeneous
        bMc = bmc.homogeneous
        aMc = amc.homogeneous
        cMa = np.linalg.inv(aMc)

        self.assertTrue(np.allclose(aMb*bMc, aMc))
        self.assertTrue(np.allclose(cma.homogeneous,cMa))

    def test_internal_product_vs_action(self):
        amb = se3.SE3.Random()
        bmc = se3.SE3.Random()
        amc = amb * bmc
        cma = amc.inverse()
        
        aXb = amb.action
        bXc = bmc.action
        aXc = amc.action
        cXa = np.linalg.inv(aXc)

        self.assertTrue(np.allclose(aXb*bXc, aXc))
        self.assertTrue(np.allclose(cma.action,cXa))

    def test_point_action(self):
        amb = se3.SE3.Random()
        aMb = amb.homogeneous
        p_homogeneous = rand(4)
        p_homogeneous[3] = 1
        p = p_homogeneous[0:3].copy()

        # act
        self.assertTrue(np.allclose(amb.act(p),(aMb*p_homogeneous)[0:3]))

        # actinv
        bMa = np.linalg.inv(aMb)
        bma = amb.inverse()
        self.assertTrue(np.allclose(bma.act(p), (bMa * p_homogeneous)[0:3]))

if __name__ == '__main__':
    unittest.main()
