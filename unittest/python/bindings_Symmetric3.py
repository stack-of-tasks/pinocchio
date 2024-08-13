import unittest
import pinocchio as pin
import numpy as np
from pinocchio.utils import eye, zero

from test_case import PinocchioTestCase as TestCase


class TestSymmetric3Bindings(TestCase):
    def test_zero_getter(self):
        S = pin.Symmetric3.Zero()
        self.assertTrue(np.allclose(zero(6), S.data))

    def test_identity_getters_matrix_conversion(self):
        S = pin.Symmetric3.Identity()
        id_vec = np.array([1.0, 0.0, 1.0, 0.0, 0.0, 1.0])
        id_matrix = pin.Symmetric3(id_vec).matrix()
        self.assertTrue(np.allclose(eye(3), id_matrix))
        self.assertTrue(np.allclose(np.array([1.0, 0.0, 1.0, 0.0, 0.0, 1.0]), S.data))
        self.assertTrue(np.allclose(eye(3), S.matrix()))

    def test_setRandom_matrix_conversion(self):
        S = pin.Symmetric3.Identity()
        S.setRandom()
        self.assertFalse(np.allclose(eye(3), S.matrix()))
        S_matrix = S.matrix()
        tri_upper = np.triu(S_matrix, k=1)
        tri_lower = np.tril(S_matrix, k=-1)
        self.assertTrue(np.allclose(tri_upper, tri_lower.T))

    def test_setZero(self):
        S = pin.Symmetric3.Zero()
        S.setRandom()
        S.setZero()
        self.assertTrue(np.allclose(zero(6), S.data))

    def test_setIdentity_matrix_conversion(self):
        S = pin.Symmetric3.Zero()
        S.setIdentity()
        self.assertTrue(np.allclose(eye(3), S.matrix()))

    def test_setDiagonal(self):
        S = pin.Symmetric3.Zero()
        S.setDiagonal(np.ones(3) * 2)
        self.assertTrue(np.allclose(eye(3) * 2, S.matrix()))


if __name__ == "__main__":
    unittest.main()
