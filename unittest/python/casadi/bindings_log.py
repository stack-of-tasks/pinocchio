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
    """Test the exp map and its derivative."""
    dw = SX.sym("dw", 3)  # for forming the difference
    exp_expr = self.cR0_i @ cpin.exp3(self.w + dw)
    repl_dargs = lambda e: casadi.substitute(e, casadi.vertcat(self.dv0, dw), np.zeros(6))

    exp_eval = casadi.Function("exp", [self.cR0, self.w], [repl_dargs(exp_expr)])

    diff_expr = cpin.log3(exp_eval(self.cR0, self.w).T @ exp_expr)
    Jexp_expr = casadi.jacobian(diff_expr, casadi.vertcat(self.dv0, dw))
    Jexp_eval = casadi.Function("exp", [self.cR0, self.w], [repl_dargs(Jexp_expr)])

    np.random.seed(3)
    R0 = np.eye(3)
    w1 = np.random.randn(3)
    w2 = np.array([np.pi, 0, 0])
    R1 = pin.exp3(w1)
    R2 = pin.exp3(w2)
    self.assertApprox(exp_eval(R0, np.zeros(3)).full(), R0)
    self.assertApprox(exp_eval(R0, w1).full(), R1)
    self.assertApprox(exp_eval(R0, -w1).full(), R1.T)
    self.assertApprox(exp_eval(R1, -w1).full(), R0)
    self.assertApprox(exp_eval(R0, w2).full(), R2)

    w0 = np.zeros(3)
    J0 = pin.Jexp3(w0)
    J1 = pin.Jexp3(w1)
    J2 = pin.Jexp3(w2)
    # print(J0)
    # print(Jexp_eval(R0, np.zeros(3)))
    # print(J1)
    # print(Jexp_eval(R0, w1))
    # print("R1:", R1)
    # print(Jexp_eval(R1, w1))
    # print(Jexp_eval(R2, w1))
    self.assertApprox(Jexp_eval(R0, w0).full(), np.hstack([np.eye(3), J0]))
    self.assertApprox(Jexp_eval(R1, w0).full(), np.hstack([np.eye(3), J0]))
    self.assertApprox(Jexp_eval(R0, w1).full(), np.hstack([R1.T, J1]))
    self.assertApprox(Jexp_eval(R1, w2).full(), np.hstack([R2.T, J2]))
    self.assertApprox(Jexp_eval(R0, w2).full(), np.hstack([R2.T, J2]))


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
    R2 = pin.exp3(np.array([np.pi, 0, 0]))

    self.assertApprox(log_eval(R0, R0).full().squeeze(), np.zeros(3))
    self.assertApprox(log_eval(R0, R1).full().squeeze(), vr)
    self.assertApprox(log_eval(R1, R1).full().squeeze(), np.zeros(3))
    self.assertApprox(log_eval(R0, R2).full().squeeze(), np.array([np.pi, 0, 0]))

    jac_identity = np.hstack([-np.eye(3), np.eye(3)])
    self.assertApprox(Jlog_eval(R0, R0).full(), jac_identity)

    J_1 = pin.Jlog3(R1)
    J_0 = -R1.T @ J_1
    self.assertApprox(Jlog_eval(R0, R1).full(), np.hstack([J_0, J_1]))


if __name__ == '__main__':
  unittest.main()
