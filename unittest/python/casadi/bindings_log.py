import unittest
import sys, os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from test_case import PinocchioTestCase as TestCase

import pinocchio as pin
import pinocchio.casadi as cpin

import casadi
from casadi import SX
import numpy as np


class TestLogExpDerivatives(TestCase):

    def setUp(self) -> None:
        self.cR0 = SX.sym("R0", 3, 3)
        self.cR1 = SX.sym("R1", 3, 3)
        self.dv0 = SX.sym("v0", 3)
        self.dv1 = SX.sym("v1", 3)
        self.v_all = casadi.vertcat(self.dv0, self.dv1)
        self.w = SX.sym("v", 3)

        self.cR0_i = self.cR0 @ cpin.exp3(self.dv0)
        self.cR1_i = self.cR1 @ cpin.exp3(self.dv1)

    def test_exp3(self):
        exp_expr = self.cR0_i @ cpin.exp3(self.w)
        exp_eval = casadi.Function("exp", [self.cR0, self.w],
                                   [casadi.substitute(exp_expr, self.dv0, np.zeros(3))])

        Jexp_expr = casadi.jacobian(exp_expr, casadi.vertcat(self.dv0, self.w))
        Jexp_eval = casadi.Function("exp", [self.cR0, self.w],
                                   [casadi.substitute(Jexp_expr, self.dv0, np.zeros(3))])

        R0 = np.eye(3)
        v1 = np.random.randn(3)
        R1 = pin.exp3(v1)
        self.assertApprox(exp_eval(R0, np.zeros(3)).full(), R0)
        self.assertApprox(exp_eval(R0, v1).full(), R1)
        self.assertApprox(exp_eval(R0, -v1).full(), R1.T)
        self.assertApprox(exp_eval(R1, -v1).full(), np.eye(3))


    def test_log3(self):
        log_expr = cpin.log3(self.cR0_i.T @ self.cR1_i)
        log_eval = casadi.Function("log", [self.cR0, self.cR1],
                                   [casadi.substitute(log_expr, self.v_all, np.zeros(6))])

        Jlog_expr = casadi.jacobian(log_expr, self.v_all)
        Jlog_eval = casadi.Function("Jlog", [self.cR0, self.cR1],
                                    [casadi.substitute(Jlog_expr, self.v_all, np.zeros(6))])

        R0 = np.eye(3)
        vr = np.random.randn(3)
        R1 = pin.exp3(vr)

        self.assertApprox(log_eval(R0, R0).full(), np.zeros(3))
        self.assertApprox(log_eval(R0, R1).full(), vr)
        self.assertApprox(log_eval(R1, R1).full(), np.zeros(3))

        jac_identity = np.hstack([-np.eye(3), np.eye(3)])
        self.assertApprox(Jlog_eval(R0, R0).full(), jac_identity)

        J_1 = pin.Jlog3(R1)
        J_0 = -R1.T @ J_1
        self.assertApprox(Jlog_eval(R0, R1).full(), np.hstack([J_0, J_1]))


if __name__ == '__main__':
    unittest.main()
