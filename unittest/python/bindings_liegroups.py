import unittest
import pinocchio as pin
import numpy as np
from pinocchio.utils import eye,zero,rand

ones = lambda n: np.ones([n, 1] if isinstance(n, int) else n)

class TestLiegroupBindings(unittest.TestCase):

    def test_basic(self):
        R3 = pin.liegroups.R3()
        SO3 = pin.liegroups.SO3()
        R3xSO3 = R3 * SO3

        self.assertEqual(R3.name, "R^3")
        self.assertEqual(SO3.name, "SO(3)")
        self.assertEqual(R3xSO3.name, R3.name + " x " + SO3.name)

        self.assertEqual(R3.nq+SO3.nq, R3xSO3.nq)
        self.assertEqual(R3.nv+SO3.nv, R3xSO3.nv)

    def test_dIntegrate(self):
        for lg in [ pin.liegroups.R3(),
                    pin.liegroups.SO3(),
                    pin.liegroups.SE3(),
                    pin.liegroups.R3() * pin.liegroups.SO3(),
                    ]:
            q = lg.random()
            v = np.random.rand(lg.nv)

            J = lg.dIntegrate_dq(q, v)
            #self.assertTrue(np.allclose(np.eye(3),J))

            SELF = 0
            J0 = np.random.rand(lg.nv,lg.nv)

            J1 = lg.dIntegrate_dq(q, v, SELF, J0)
            self.assertTrue(np.allclose(J @ J0,J1))

            J1 = lg.dIntegrate_dq(q, v, J0, SELF)
            self.assertTrue(np.allclose(J0 @ J,J1))

if __name__ == '__main__':
    unittest.main()
