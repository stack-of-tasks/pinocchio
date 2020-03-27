import unittest
import pinocchio as pin

from test_case import PinocchioTestCase as TestCase

class TestData(TestCase):
    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()
        self.data = self.model.createData()

    def test_copy(self):
        data2 = self.data.copy()
        q = pin.neutral(self.model)
        pin.forwardKinematics(self.model,data2,q)
        jointId = self.model.njoints-1
        self.assertNotEqual(self.data.oMi[jointId], data2.oMi[jointId])

        data3 = data2.copy()
        self.assertEqual(data2.oMi[jointId], data3.oMi[jointId])

    def test_pickle(self):
        import pickle

        data = self.data
        filename = "data.pickle"
        with open(filename, 'wb') as f:
          pickle.dump(data,f)
    
        with open(filename, 'rb') as f:
          data_copy = pickle.load(f)

        self.assertTrue(data == data_copy)

if __name__ == '__main__':
    unittest.main()
