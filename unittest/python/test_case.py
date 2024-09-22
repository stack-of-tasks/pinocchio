import unittest

from pinocchio.utils import isapprox


def tracefunc(frame, event, arg):
    print("%s, %s: %d" % (event, frame.f_code.co_filename, frame.f_lineno))
    return tracefunc


class PinocchioTestCase(unittest.TestCase):
    def assertApprox(self, a, b, eps=1e-6):
        return self.assertTrue(
            isapprox(a, b, eps),
            f"\n{a}\nis not approximately equal to\n{b}\nwith precision {eps:f}",
        )
