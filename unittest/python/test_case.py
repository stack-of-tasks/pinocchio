import unittest

from pinocchio.utils import isapprox

def tracefunc(frame, event, arg):
    print("%s, %s: %d" % (event, frame.f_code.co_filename, frame.f_lineno))
    return tracefunc

class PinocchioTestCase(unittest.TestCase):
    def assertApprox(self, a, b, eps=1e-6):
        return self.assertTrue(isapprox(a, b, eps),
                               "\n%s\nis not approximately equal to\n%s\nwith precision %f" % (a, b, eps))
