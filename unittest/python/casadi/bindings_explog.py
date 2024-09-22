import os
import sys
import unittest

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import casadi
import numpy as np
import pinocchio as pin
import pinocchio.casadi as cpin
from casadi import SX
from test_case import PinocchioTestCase as TestCase


class TestLogExpDerivatives(TestCase):
    def setUp(self) -> None:
        self.cR0 = SX.sym("R0", 3, 3)
        self.cR1 = SX.sym("R1", 3, 3)
        self.dv0 = SX.sym("v0", 3)
        self.dv1 = SX.sym("v1", 3)
        self.v_all = casadi.vertcat(self.dv0, self.dv1)
        self.cv2 = SX.sym("v2", 3)
        self.cdv2 = SX.sym("dv", 3)  # for forming the difference

        self.cR0_i = self.cR0 @ cpin.exp3(self.dv0)
        self.cR1_i = self.cR1 @ cpin.exp3(self.dv1)

        # SE(3) examples
        self.cM0 = SX.sym("M0", 4, 4)
        self.cM1 = SX.sym("M1", 4, 4)
        self.cdw0 = SX.sym("dm0", 6)
        self.cdw1 = SX.sym("dm1", 6)
        self.cw2 = SX.sym("w2", 6)
        self.cdw2 = SX.sym("dw2", 6)
        self.cM0_i = cpin.SE3(self.cM0) * cpin.exp6(self.cdw0)
        self.cM1_i = cpin.SE3(self.cM1) * cpin.exp6(self.cdw1)

    def test_exp3(self):
        """Test the exp map and its derivative."""
        dw = self.cdv2
        exp_expr = self.cR0_i @ cpin.exp3(self.cv2 + dw)

        def repl_dargs(e):
            return casadi.substitute(e, casadi.vertcat(self.dv0, dw), np.zeros(6))

        exp_eval = casadi.Function("exp", [self.cR0, self.cv2], [repl_dargs(exp_expr)])

        diff_expr = cpin.log3(exp_eval(self.cR0, self.cv2).T @ exp_expr)
        Jexp_expr = casadi.jacobian(diff_expr, casadi.vertcat(self.dv0, dw))
        Jexp_eval = casadi.Function(
            "exp", [self.cR0, self.cv2], [repl_dargs(Jexp_expr)]
        )

        w0 = np.zeros(3)
        w1 = np.random.randn(3)
        w2 = np.array([np.pi, 0, 0])
        w3 = np.array([-np.pi, 0, 0])
        R0 = pin.exp3(w0)  # eye(3)
        R1 = pin.exp3(w1)
        R2 = pin.exp3(w2)
        R3 = pin.exp3(w3)
        self.assertApprox(exp_eval(R0, np.zeros(3)).full(), R0)
        self.assertApprox(exp_eval(R0, w1).full(), R1)
        self.assertApprox(exp_eval(R0, -w1).full(), R1.T)
        self.assertApprox(exp_eval(R1, -w1).full(), R0)
        self.assertApprox(exp_eval(R0, w2).full(), R2)

        J0 = pin.Jexp3(w0)
        J1 = pin.Jexp3(w1)
        J2 = pin.Jexp3(w2)
        J3 = pin.Jexp3(w3)
        # print(J0)
        # print(Jexp_eval(R0, np.zeros(3)))
        # print(J1)
        # print(Jexp_eval(R0, w1))
        # print("R1:", R1)
        # print(Jexp_eval(R1, w1))
        # print(Jexp_eval(R2, w1))
        self.assertApprox(Jexp_eval(R0, w0).full(), np.hstack([R0.T, J0]))
        self.assertApprox(Jexp_eval(R0, w1).full(), np.hstack([R1.T, J1]))
        self.assertApprox(Jexp_eval(R0, w2).full(), np.hstack([R2.T, J2]))
        self.assertApprox(Jexp_eval(R0, w3).full(), np.hstack([R3.T, J3]))
        self.assertApprox(Jexp_eval(R1, w0).full(), np.hstack([R0.T, J0]))
        self.assertApprox(Jexp_eval(R1, w2).full(), np.hstack([R2.T, J2]))

    def test_log3(self):
        log_expr = cpin.log3(self.cR0_i.T @ self.cR1_i)
        log_eval = casadi.Function(
            "log",
            [self.cR0, self.cR1],
            [casadi.substitute(log_expr, self.v_all, np.zeros(6))],
        )

        Jlog_expr = casadi.jacobian(log_expr, self.v_all)
        Jlog_eval = casadi.Function(
            "Jlog",
            [self.cR0, self.cR1],
            [casadi.substitute(Jlog_expr, self.v_all, np.zeros(6))],
        )

        R0 = np.eye(3)
        vr = np.random.randn(3)
        R1 = pin.exp3(vr)
        R2 = pin.exp3(np.array([np.pi, 0, 0]))
        v3 = np.array([0, np.pi, 0])
        R3 = pin.exp3(v3)

        self.assertApprox(log_eval(R0, R0).full().squeeze(), np.zeros(3))
        self.assertApprox(log_eval(R0, R1).full().squeeze(), vr)
        self.assertApprox(log_eval(R1, R1).full().squeeze(), np.zeros(3))
        self.assertApprox(log_eval(R0, R2).full().squeeze(), np.array([np.pi, 0, 0]))
        self.assertApprox(log_eval(R0, R3).full().squeeze(), v3)

        J0 = pin.Jlog3(R0)
        jac_identity = np.hstack([-J0, J0])
        self.assertApprox(Jlog_eval(R0, R0).full(), jac_identity)

        J1 = pin.Jlog3(R1)
        self.assertApprox(Jlog_eval(R0, R1).full(), np.hstack([-R1.T @ J1, J1]))

        J2 = pin.Jlog3(R2)
        self.assertApprox(Jlog_eval(R0, R2).full(), np.hstack([-R2.T @ J2, J2]))

    def test_log3_quat(self):
        cquat = SX.sym("quat", 4)
        cdv = SX.sym("dv", 3)
        SO3 = cpin.liegroups.SO3()

        def repl_dargs(e):
            return casadi.substitute(e, cdv, np.zeros(3))

        cquat_i = SO3.integrate(cquat, cdv)

        clog = cpin.log3(cquat_i)
        cJlog = casadi.jacobian(clog, cdv)
        clog_eval = casadi.Function("log", [cquat], [repl_dargs(clog)])
        cJlog_eval = casadi.Function("Jlog", [cquat], [repl_dargs(cJlog)])

        q0 = np.array([0.0, 0.0, 0.0, 1.0])
        q1 = np.array([0.0, 1.0, 0.0, 0.0])
        q2 = np.array([0.0, 0.0, 1.0, 0.0])
        q3 = np.array([1.0, 0.0, 0.0, 0.0])

        self.assertApprox(clog_eval(q0).full().squeeze(), np.zeros(3))
        self.assertApprox(cJlog_eval(q0).full().squeeze(), np.eye(3))

        clog_fun = casadi.dot(clog, clog)
        cJlog_fun = casadi.jacobian(clog_fun, cdv)
        clog_fun_eval = casadi.Function("normlog", [cquat], [repl_dargs(clog_fun)])
        cJlog_fun_eval = casadi.Function("Jnormlog", [cquat], [repl_dargs(cJlog_fun)])

        self.assertApprox(clog_fun_eval(q0).full().squeeze(), np.zeros(1))
        self.assertApprox(cJlog_fun_eval(q0).full(), np.zeros(3))

        self.assertApprox(
            cJlog_fun_eval(q1).full(), 2 * np.pi * np.array([0.0, 1.0, 0.0])
        )
        self.assertApprox(
            cJlog_fun_eval(q2).full(), 2 * np.pi * np.array([0.0, 0.0, 1.0])
        )
        self.assertApprox(
            cJlog_fun_eval(q3).full(), 2 * np.pi * np.array([1.0, 0.0, 0.0])
        )
        print("log3 quat done")

    def test_exp6(self):
        exp_expr = cpin.exp6(self.cw2 + self.cdw2)

        def repl_dargs(e):
            return casadi.substitute(e, self.cdw2, np.zeros(6))

        exp_eval = casadi.Function("exp6", [self.cw2], [repl_dargs(exp_expr.np)])

        diff_expr = cpin.log6(cpin.SE3(exp_eval(self.cw2)).actInv(exp_expr)).np
        Jexp_expr = casadi.jacobian(diff_expr, self.cdw2)

        Jexp_eval = casadi.Function("exp6", [self.cw2], [repl_dargs(Jexp_expr)])

        w0 = np.zeros(6)
        w1 = np.array([0.0, 0.0, 0.0, np.pi, 0.0, 0.0])
        w2 = np.random.randn(6)
        w3 = np.array([0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0])
        M0 = pin.exp6(w0)
        M1 = pin.exp6(w1)
        M2 = pin.exp6(w2)
        M3 = pin.exp6(w3)
        self.assertApprox(exp_eval(w0).full(), M0.np)
        self.assertApprox(exp_eval(w1).full(), M1.np)
        self.assertApprox(exp_eval(w2).full(), M2.np)
        self.assertApprox(exp_eval(w3).full(), M3.np)

        np.set_printoptions(precision=3)
        J0 = pin.Jexp6(w0)
        self.assertApprox(Jexp_eval(w0).full(), J0)
        J1 = pin.Jexp6(w1)
        self.assertApprox(Jexp_eval(w1).full(), J1)
        J2 = pin.Jexp6(w2)
        self.assertApprox(Jexp_eval(w2).full(), J2)

    def test_log6(self):
        log_expr = cpin.log6(self.cM0_i.actInv(self.cM1_i))

        def repl_dargs(e):
            return casadi.substitute(
                e, casadi.vertcat(self.cdw0, self.cdw1), np.zeros(12)
            )

        log_eval = casadi.Function(
            "log6", [self.cM0, self.cM1], [repl_dargs(log_expr.np)]
        )

        Jlog_expr = casadi.jacobian(log_expr.np, casadi.vertcat(self.cdw0, self.cdw1))
        Jlog_eval = casadi.Function(
            "Jlog6", [self.cM0, self.cM1], [repl_dargs(Jlog_expr)]
        )

        w0 = np.zeros(6)
        M0 = pin.exp6(np.zeros(6))
        w1 = np.array([0, 0, 0, np.pi, 0.0, 0.0])
        M1 = pin.exp6(w1)
        w2 = np.random.randn(6)
        M2 = pin.exp6(w2)

        self.assertApprox(log_eval(M0.np, M0.np).full(), w0)
        self.assertApprox(log_eval(M1.np, M1.np).full(), w0)
        self.assertApprox(log_eval(M0.np, M1.np).full(), w1)
        self.assertApprox(log_eval(M0.np, M2.np).full(), w2)

        J0 = pin.Jlog6(M0)
        J1 = pin.Jlog6(M1)
        J2 = pin.Jlog6(M2)
        self.assertApprox(Jlog_eval(M0.np, M0.np).full(), np.hstack([-J0, J0]))
        self.assertApprox(
            Jlog_eval(M0.np, M1.np).full(), np.hstack([-M1.dualAction.T @ J1, J1])
        )
        self.assertApprox(
            Jlog_eval(M0.np, M2.np).full(), np.hstack([-M2.dualAction.T @ J2, J2])
        )

    def test_log6_quat(self):
        cq0 = SX.sym("q0", 7)
        cv0 = SX.sym("q0", 6)

        def repl_dargs(e):
            return casadi.substitute(e, cv0, np.zeros(6))

        SE3 = cpin.liegroups.SE3()

        cq0_i = SE3.integrate(cq0, cv0)
        clog = cpin.log6_quat(cq0_i).vector
        clog_eval = casadi.Function("log", [cq0], [repl_dargs(clog)])

        cJlog = casadi.jacobian(clog, cv0)
        cJlog_eval = casadi.Function("Jlog", [cq0], [repl_dargs(cJlog)])

        q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
        q2 = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        q3 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.assertApprox(clog_eval(q0).full().squeeze(), np.zeros(6))
        self.assertApprox(cJlog_eval(q0).full(), np.eye(6))

        print("log6 with quats")
        print(clog_eval(q1).full())
        print(cJlog_eval(q1).full())

        clog_fun = casadi.dot(clog, clog)
        clog_fun = casadi.dot(clog, clog)
        cJlog_fun = casadi.jacobian(clog_fun, cv0)
        clog_fun_eval = casadi.Function("normlog", [cq0], [repl_dargs(clog_fun)])
        cJlog_fun_eval = casadi.Function("Jnormlog", [cq0], [repl_dargs(cJlog_fun)])

        print(clog_fun_eval(q0).full().squeeze())
        print(cJlog_fun_eval(q0).full())

        print(clog_fun_eval(q1).full().squeeze())
        print(cJlog_fun_eval(q1).full())

        print(clog_fun_eval(q2).full().squeeze())
        print(cJlog_fun_eval(q2).full())

        print(clog_fun_eval(q3).full().squeeze())
        print(cJlog_fun_eval(q3).full())


if __name__ == "__main__":
    unittest.main()
