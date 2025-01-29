import unittest

import pinocchio as pin


class TestCoalTransformConversion(unittest.TestCase):
    def test_from_SE3(self):
        M = pin.SE3.Random()
        coal_transform = pin.coal.Transform3f(M)

        self.assertTrue((M.rotation == coal_transform.getRotation()).all())
        self.assertTrue((M.translation == coal_transform.getTranslation()).all())

    def test_to_SE3(self):
        coal_transform = pin.coal.Transform3f()
        M = pin.SE3(coal_transform)
        self.assertTrue(M.isIdentity())


if __name__ == "__main__":
    if pin.WITH_COAL_BINDINGS:
        unittest.main()
