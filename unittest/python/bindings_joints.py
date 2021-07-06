import unittest
import pinocchio as pin
import numpy as np

joint_types = [
    pin.JointModelRX, pin.JointModelRY, pin.JointModelRZ,
    pin.JointModelPX, pin.JointModelPY, pin.JointModelPZ,
    pin.JointModelFreeFlyer, pin.JointModelSpherical, pin.JointModelSphericalZYX,
    pin.JointModelPlanar, pin.JointModelTranslation
]

class TestJoints(unittest.TestCase):

    def test_comparison_operators(self):
        for joint_type in joint_types:

            j = joint_type()
            joint_model = pin.JointModel(j)

            self.assertTrue(j == joint_type())
            self.assertTrue(j == joint_model)

            j.setIndexes(0,0,0)
            self.assertFalse(j == joint_model)
            self.assertTrue(j != joint_model)

if __name__ == '__main__':
    unittest.main()
