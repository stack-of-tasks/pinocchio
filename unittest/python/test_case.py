import unittest

from pinocchio.utils import isapprox


class TestCase(unittest.TestCase):
    def assertApprox(self, a, b):
        return self.assertTrue(isapprox(a, b))
