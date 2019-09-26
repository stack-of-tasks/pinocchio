import unittest

import pinocchio as pin
pin.switchToNumpyMatrix()
from pinocchio.utils import isapprox

def tracefunc(frame, event, arg):
    print("%s, %s: %d" % (event, frame.f_code.co_filename, frame.f_lineno))
    return tracefunc

class TestCase(unittest.TestCase):
    def assertApprox(self, a, b):
        return self.assertTrue(isapprox(a, b))
