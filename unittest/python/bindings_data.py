import unittest
from pathlib import Path

import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


class TestData(TestCase):
    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

    def test_copy(self):
        data2 = self.data.copy()
        q = pin.neutral(self.model)
        pin.forwardKinematics(self.model, data2, q)
        jointId = self.model.njoints - 1
        self.assertNotEqual(self.data.oMi[jointId], data2.oMi[jointId])

        data3 = data2.copy()
        self.assertEqual(data2.oMi[jointId], data3.oMi[jointId])

    def test_std_vector_field(self):
        model = self.model
        data = self.data

        q = pin.neutral(model)
        pin.centerOfMass(model, data, q)

        _com_list = data.com.tolist()
        com = data.com[0]
        with self.assertRaises(Exception) as context:
            com = data.com[len(data.com) + 10]
            print("com: ", com)

        self.assertTrue("Index out of range" in str(context.exception))

        with self.assertRaises(Exception) as context:
            com = data.com["1"]
            print("com: ", com)

        self.assertTrue("Invalid index type" in str(context.exception))

    def test_pickle(self):
        import pickle

        data = self.data
        filename = Path("data.pickle")
        with filename.open("wb") as f:
            pickle.dump(data, f)

        with filename.open("rb") as f:
            data_copy = pickle.load(f)

        self.assertTrue(data == data_copy)


if __name__ == "__main__":
    unittest.main()
