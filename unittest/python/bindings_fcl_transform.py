import unittest

import pinocchio as pin


class TestFCLTransformConversion(unittest.TestCase):
    def test_from_SE3(self):
        M = pin.SE3.Random()
        fcl_transform = pin.coal.Transform3f(M)

        self.assertTrue((M.rotation == fcl_transform.getRotation()).all())
        self.assertTrue((M.translation == fcl_transform.getTranslation()).all())

    def test_to_SE3(self):
        fcl_transform = pin.coal.Transform3f()
        M = pin.SE3(fcl_transform)
        self.assertTrue(M.isIdentity())


if __name__ == "__main__":
    if pin.WITH_COAL_BINDINGS:
        unittest.main()
