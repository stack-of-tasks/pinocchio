import unittest
import pinocchio as pin

class TestGeometryModelBindings(unittest.TestCase):

    def test_pair_equals(self):
        c1 = pin.CollisionPair(1,2)
        c2 = pin.CollisionPair(1,2)
        c3 = pin.CollisionPair(3,4)

        self.assertEqual(c1,c2)
        self.assertTrue(c1==c2)
        self.assertFalse(c1!=c2)

        self.assertNotEqual(c1,c3)
        self.assertTrue(c1!=c3)
        self.assertFalse(c1==c3)
    
    def test_pair_copy(self):
        c1 = pin.CollisionPair(1,2)
        c2 = c1.copy()

        self.assertEqual(c1,c2)

        c2.second = 3
        self.assertNotEqual(c1,c2)

if __name__ == '__main__':
    unittest.main()
