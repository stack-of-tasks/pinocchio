from __future__ import print_function

import sys
import unittest
import logging

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

    @unittest.skipUnless(pin.WITH_FCL_SUPPORT(),"Needs FCL")
    def test_collision_pairs(self):
        log = logging.getLogger("log."+self.id())
        log.debug('\n')

        c0 = pin.CollisionPair(1,0)
        c1 = pin.CollisionPair(2,0)
        c2 = pin.CollisionPair(3,0)

        c0c = c0.copy()
        c1c = c1.copy()
        c2c = c2.copy()

        gmodel = pin.GeometryModel()
        gmodel.addCollisionPair(c0)
        gmodel.addCollisionPair(c1)
        gmodel.addCollisionPair(c2)

        self.assertEqual(len(gmodel.collisionPairs),3)
        log.debug(gmodel.collisionPairs[0]) # (1,0)
        self.assertEqual(gmodel.collisionPairs[0],c0c)
        log.debug(gmodel.collisionPairs[1]) # (2,0)
        self.assertEqual(gmodel.collisionPairs[1],c1c)
        log.debug(gmodel.collisionPairs[2]) # (3,0)
        self.assertEqual(gmodel.collisionPairs[2],c2c)

        p = gmodel.collisionPairs[1]
        log.debug(p) # (2,0)
        self.assertEqual(p,c1c)

        gmodel.removeCollisionPair(gmodel.collisionPairs[0])
        log.debug(p) # still (2,0)
        self.assertEqual(p,c1c)

        for c in gmodel.collisionPairs:
            self.assertNotEqual(c,c0)

        self.assertEqual(len(gmodel.collisionPairs),2)
        log.debug(gmodel.collisionPairs[0]) # (2,0)
        self.assertEqual(gmodel.collisionPairs[0],c1c)
        log.debug(gmodel.collisionPairs[1]) # (3,0)
        self.assertEqual(gmodel.collisionPairs[1],c2c)

        c3 = pin.CollisionPair(4,0)
        c3c = c3.copy()

        gmodel.addCollisionPair(c3)

        self.assertEqual(len(gmodel.collisionPairs),3)
        log.debug(gmodel.collisionPairs[0]) # (2,0)
        self.assertEqual(gmodel.collisionPairs[0],c1c)
        log.debug(gmodel.collisionPairs[1]) # (3,0)
        self.assertEqual(gmodel.collisionPairs[1],c2c)
        log.debug(gmodel.collisionPairs[2]) # (4,0)
        self.assertEqual(gmodel.collisionPairs[2],c3c)

        c4 = pin.CollisionPair(5,0)
        c4c = c4.copy()

        gmodel.collisionPairs[2] = c4
        log.debug(gmodel.collisionPairs[2]) # (5,0)
        self.assertEqual(gmodel.collisionPairs[2],c4c)
        self.assertEqual(c3,c3c)

if __name__ == '__main__':
    if ('-v' in sys.argv) or ('--verbose' in sys.argv):
        logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    unittest.main()
