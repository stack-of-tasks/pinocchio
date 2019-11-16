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
        pickle.dump(model, open( "save.p", "wb" ) )
      
        model_copy = pickle.load( open( "save.p", "rb" ) )
        self.assertTrue(model == model_copy)

if __name__ == '__main__':
    unittest.main()
