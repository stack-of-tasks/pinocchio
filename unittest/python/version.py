from __future__ import print_function

import unittest
import pinocchio as pin

from test_case import PinocchioTestCase as TestCase

class TestVersion(TestCase):
    def test_version(self):
      print("version:",pin.__version__)
      print("raw_version:",pin.__raw_version__)
      self.assertTrue(pin.__version__)
      self.assertTrue(pin.__raw_version__)

if __name__ == '__main__':
    unittest.main()
