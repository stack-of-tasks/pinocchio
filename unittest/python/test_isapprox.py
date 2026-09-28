import unittest

import numpy as np
from pinocchio.utils import isapprox


class TestIsApprox(unittest.TestCase):
    def test_array_uses_the_same_absolute_tolerance_as_a_scalar(self):
        self.assertFalse(isapprox(1e6, 1e6 + 0.5))
        self.assertFalse(isapprox(np.array([1e6]), np.array([1e6 + 0.5])))
        self.assertFalse(isapprox([1e6], [1e6 + 0.5]))

    def test_values_inside_the_tolerance_still_match(self):
        self.assertTrue(isapprox(0.0, 1e-7))
        self.assertTrue(isapprox(np.array([0.0]), np.array([1e-7])))
        self.assertTrue(isapprox(np.array([1.0, 2.0]), np.array([1.0, 2.0])))


if __name__ == "__main__":
    unittest.main()
