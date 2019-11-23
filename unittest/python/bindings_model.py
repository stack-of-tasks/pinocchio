import unittest
import pinocchio as pin
pin.switchToNumpyMatrix()

from test_case import TestCase

class TestModel(TestCase):
    def setUp(self):
        self.model = pin.buildSampleModelHumanoidRandom()

    def test_pickle(self):
        import pickle

        model = self.model
        filename = "model.pickle"
        with open(filename, 'wb') as f:
          pickle.dump(model,f)
      
        with open(filename, 'rb') as f:
          model_copy = pickle.load(f)

        self.assertTrue(model == model_copy)

if __name__ == '__main__':
    unittest.main()
