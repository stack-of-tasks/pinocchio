import unittest
import pinocchio as se3
import numpy as np
from pinocchio.utils import eye,zero,rand

ones = lambda n: np.matrix(np.ones([n, 1] if isinstance(n, int) else n), np.double)

class TestSE3Bindings(unittest.TestCase):

    v3zero = zero(3)
    v3ones = ones(3)
    m3zero = zero([3,3])
    m3ones = eye(3)
    m4ones = eye(4)

    def test_identity(self):
        transform = se3.SE3.Identity()
        self.assertTrue(np.allclose(self.v3zero,transform.translation))
        self.assertTrue(np.allclose(self.m3ones, transform.rotation))
        self.assertTrue(np.allclose(self.m4ones, transform.homogeneous))
        transform.setRandom()
        transform.setIdentity()
        self.assertTrue(np.allclose(self.m4ones, transform.homogeneous))

    def test_get_translation(self):
        transform = se3.SE3.Identity()
        self.assertTrue(np.allclose(transform.translation, self.v3zero))

    def test_get_rotation(self):
        transform = se3.SE3.Identity()
        self.assertTrue(np.allclose(transform.rotation, self.m3ones))

    def test_set_translation(self):
        transform = se3.SE3.Identity()
        transform.translation = self.v3ones
        self.assertFalse(np.allclose(transform.translation, self.v3zero))
        self.assertTrue(np.allclose(transform.translation, self.v3ones))

    def test_set_rotation(self):
        transform = se3.SE3.Identity()
        transform.rotation = self.m3zero
        self.assertFalse(np.allclose(transform.rotation, self.m3ones))
        self.assertTrue(np.allclose(transform.rotation, self.m3zero))

    def test_homogeneous(self):
        amb = se3.SE3.Random()
        aMb = amb.homogeneous
        self.assertTrue(np.allclose(aMb[0:3,0:3], amb.rotation)) # bloc 33 = rotation de amb
        self.assertTrue(np.allclose(aMb[0:3,3], amb.translation)) # derniere colonne = translation de amb

    def test_inverse(self):
        amb = se3.SE3.Random()
        aMb = amb.homogeneous
        bma = amb.inverse()
        self.assertTrue(np.allclose(np.linalg.inv(aMb), bma.homogeneous))

    def test_internal_product(self):
        amb = se3.SE3.Random()
        aMb = amb.homogeneous
        bmc = se3.SE3.Random()
        bMc = bmc.homogeneous
        amc = amb*bmc
        cma = amc.inverse()
        aMc = amc.homogeneous

        self.assertTrue(np.allclose(aMb*bMc, aMc))
        self.assertTrue(np.allclose(cma.homogeneous,np.linalg.inv(aMc)))

    def test_point_action(self):
        amb = se3.SE3.Random()
        aMb = amb.homogeneous
        p_homogeneous = rand(4)
        p_homogeneous[3] = 1
        p = p_homogeneous[0:3].copy()

        # act
        self.assertTrue(np.allclose(amb.act_point(p),(aMb*p_homogeneous)[0:3]))

        # actinv
        bMa = np.linalg.inv(aMb)
        bma = amb.inverse()
        self.assertTrue(np.allclose(bma.act_point(p), (bMa * p_homogeneous)[0:3]))


    def test_action_matrix(self):
        amb = se3.SE3.Random()
        bmc = se3.SE3.Random()
        amc = amb * bmc
        aXb = amb.action
        bXc = bmc.action
        aXc = aXb * bXc
        self.assertTrue(np.allclose(amc.action,aXc))

