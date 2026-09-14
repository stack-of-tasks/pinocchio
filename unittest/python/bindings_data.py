import unittest
from pathlib import Path

import pinocchio as pin
from test_case import PinocchioTestCase as TestCase


class TestData(TestCase):
    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom(True, True)
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

        _com_list = list(data.com)
        com = data.com[0]
        with self.assertRaises(IndexError):
            com = data.com[len(data.com) + 10]
            print("com: ", com)

        with self.assertRaises(TypeError):
            com = data.com["1"]
            print("com: ", com)

        self.assertTrue("Invalid index type" in str(context.exception))

    def test_allocation(self):
        data = self.data
        self.assertEqual(data.allocation, pin.DataAllocationOption.ALL)

        self.model = pin.buildSampleModelHumanoidRandom(True, True)

        data_all = self.model.createData()
        self.assertEqual(data_all.allocation, pin.DataAllocationOption.ALL)

        data_all_2 = pin.Data(self.model)
        self.assertEqual(data_all_2.allocation, pin.DataAllocationOption.ALL)

        data_no_tensors = self.model.createData(pin.DataAllocationOption.NO_TENSORS)
        self.assertEqual(
            data_no_tensors.allocation, pin.DataAllocationOption.NO_TENSORS
        )

        data_no_tensors_2 = pin.Data(self.model, pin.DataAllocationOption.NO_TENSORS)
        self.assertEqual(
            data_no_tensors_2.allocation, pin.DataAllocationOption.NO_TENSORS
        )

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
