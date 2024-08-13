import unittest

import numpy as np
import pinocchio as pin
from numpy.random import rand
from pinocchio import skew, skewSquare, unSkew

try:
    from pinocchio import casadi as cpin

    import casadi

    WITH_CASADI = True
except ImportError:
    WITH_CASADI = False

from test_case import PinocchioTestCase


class TestSpatial(PinocchioTestCase):
    def test_skew(self):
        v3 = rand(3)
        self.assertApprox(v3, unSkew(skew(v3)))
        self.assertLess(np.linalg.norm(skew(v3).dot(v3)), 1e-10)

        x, y, z = tuple(rand(3).tolist())
        M = np.array([[0.0, x, y], [-x, 0.0, z], [-y, -z, 0.0]])
        self.assertApprox(M, skew(unSkew(M)))

        rhs = rand(3)
        self.assertApprox(np.cross(v3, rhs, axis=0), skew(v3).dot(rhs))
        self.assertApprox(M.dot(rhs), np.cross(unSkew(M), rhs, axis=0))

        x, y, z = tuple(v3.tolist())
        self.assertApprox(skew(v3), np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]]))

    def skewSquare(self):
        v3 = rand(3)

        Mss = skewSquare(v3)

        Ms = skew(v3)
        Mss_ref = Ms.dot(Ms)

        self.assertApprox(Mss, Mss_ref)

    def test_NaN_log3_casadi(self):
        if WITH_CASADI:
            Rtarget = pin.utils.rotate("x", 3.14 / 4)  # Target
            R0 = pin.Quaternion(0.707107, 0.707107, 0, 0).matrix()
            nu0 = pin.log3(R0)

            # Casadi symbolic variables and functions functions
            nu = casadi.SX.sym("v", 3, 1)
            D = cpin.log3(
                cpin.exp3(nu).T @ Rtarget
            )  # This seems to be the probelematic function
            dDi_dnu = [
                casadi.Function("gradient" + str(i), [nu], [casadi.gradient(D[i], nu)])
                for i in range(3)
            ]  # Compute the gradient of function D wrt nu

            d0 = dDi_dnu[0](nu0)  # Evaluate the gradient at a problematic point nu0
            self.assertFalse(
                np.any(np.isnan(d0)), "NaN detected in the log3 function derivative"
            )

    def test_Jlog6(self):
        for _ in range(10):
            M0: pin.SE3 = pin.SE3.Random()
            M1 = M0.copy()
            dM = M0.actInv(M1)
            J = pin.Jlog6(dM)
            R = dM.rotation
            logR = pin.log3(R)
            Jrot = pin.Jlog3(R)
            print(R, ":\nlog: {}".format(logR))
            print(Jrot)

            self.assertApprox(dM, pin.SE3.Identity())
            self.assertApprox(Jrot, np.eye(3))
            self.assertApprox(J, np.eye(6))


if __name__ == "__main__":
    unittest.main()
