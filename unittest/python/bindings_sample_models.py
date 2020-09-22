import unittest
from test_case import PinocchioTestCase as TestCase

import pinocchio as pin

class TestSampleModels(TestCase):

    def setUp(self):
        pass

    def test_all_sampled_models(self):

        huamnoid_1 = pin.buildSampleModelHumanoidRandom()
        huamnoid_2 = pin.buildSampleModelHumanoidRandom(True)
        huamnoid_3 = pin.buildSampleModelHumanoidRandom(False)

        self.assertTrue(huamnoid_1 != huamnoid_2)
        self.assertTrue(huamnoid_1 != huamnoid_3)

        manipulator_1 = pin.buildSampleModelManipulator()

        if pin.WITH_HPP_FCL:
            geometry_manipulator_1 = pin.buildSampleGeometryModelManipulator(manipulator_1)

        humanoid_4 = pin.buildSampleModelHumanoid()
        humanoid_5 = pin.buildSampleModelHumanoid(True)
        humanoid_6 = pin.buildSampleModelHumanoid(False)

        self.assertTrue(humanoid_4 == humanoid_5)
        self.assertTrue(humanoid_4 != humanoid_6)

        if pin.WITH_HPP_FCL:
            geometry_humanoid_2 = pin.buildSampleGeometryModelHumanoid(humanoid_4)

if __name__ == '__main__':
    unittest.main()
