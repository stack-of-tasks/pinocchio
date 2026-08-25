"""
CasADi autodiff through the SE(3) spline joint.

The model is the parabola of examples/spline-joint.py: a unit box sliding along a
degree 3 SE(3) spline shaped like z = y**2, driven by gravity.

Because a optimization problem is solved, the spline expression are differentiated
through the CasADi interface, verifying that splines are compatible with CasADi.
"""

import sys
import unittest
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))
import casadi
import numpy as np
import pinocchio as pin
from casadi import SX
from pinocchio import casadi as cpin
from test_case import PinocchioTestCase as TestCase

# Control frames of the parabola spline, as (ty, tz).
# Same values as examples/spline-joint.py.
PARABOLA_CONTROL_FRAMES = [
    (-1.000000, 1.000000),
    (-0.888889, 0.777778),
    (-0.666667, 0.407407),
    (-0.333333, 0.074074),
    (0.000000, -0.037037),
    (0.333333, 0.074074),
    (0.666667, 0.407407),
    (0.888889, 0.777778),
    (1.000000, 1.000000),
]


MIN_Q, MAX_Q = -1.0, 1.0


def build_model(degree=3, control_frames=PARABOLA_CONTROL_FRAMES):
    """Single spline joint carrying a unit box, plus the joint model itself."""
    frames = [
        pin.SE3(np.eye(3), np.array([0.0, ty, tz])) for (ty, tz) in control_frames
    ]
    joint = (
        pin.JointModelSplineBuilder()
        .withDegree(degree)
        .withControlFrameVector(frames)
        .withOpenUniformKnots(MIN_Q, MAX_Q)
        .build()
    )
    model = pin.Model()
    joint_id = model.addJoint(0, joint, pin.SE3.Identity(), "spline")
    model.appendBodyToJoint(
        joint_id, pin.Inertia.FromBox(1.0, 1.0, 1.0, 1.0), pin.SE3.Identity()
    )
    return model, joint


class TestJointSplineCasadi(TestCase):
    def setUp(self):
        self.model, self.joint = build_model()
        self.data = self.model.createData()

        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()

        self.cq = SX.sym("q", self.cmodel.nq)
        self.cv = SX.sym("v", self.cmodel.nv)
        self.ctau = SX.sym("tau", self.cmodel.nv)

    def test_mini_ocp(self):
        """
        Slide the box from one side of the parabola to the other.
        """
        nodes, dt = 25, 0.04
        q_init, q_goal = -0.9, 0.9

        opti = casadi.Opti()
        qs = opti.variable(1, nodes + 1)
        vs = opti.variable(1, nodes + 1)
        taus = opti.variable(1, nodes)

        # Semi implicit Euler, built once as a CasADi Function and reused.
        a = cpin.aba(self.cmodel, self.cdata, self.cq, self.cv, self.ctau)
        v_next = self.cv + dt * a
        q_next = cpin.integrate(self.cmodel, self.cq, dt * v_next)
        step = casadi.Function("step", [self.cq, self.cv, self.ctau], [q_next, v_next])

        opti.minimize(casadi.sumsqr(taus))

        for i in range(nodes):
            q_i_1, v_i_1 = step(qs[i], vs[i], taus[i])
            opti.subject_to(qs[i + 1] == q_i_1)
            opti.subject_to(vs[i + 1] == v_i_1)

        opti.subject_to(qs[0] == q_init)
        opti.subject_to(vs[0] == 0.0)
        opti.subject_to(qs[nodes] == q_goal)
        opti.subject_to(vs[nodes] == 0.0)

        # Outside [min_q, max_q] the calc throws.
        margin = 1e-6
        opti.subject_to(
            opti.bounded(self.joint.min_q + margin, qs, self.joint.max_q - margin)
        )
        opti.subject_to(opti.bounded(-50.0, taus, 50.0))

        opti.set_initial(qs, np.linspace(q_init, q_goal, nodes + 1).reshape(1, -1))
        opti.set_initial(vs, (q_goal - q_init) / (nodes * dt))
        opti.set_initial(taus, 0.0)

        opti.solver(
            "ipopt",
            {"print_time": False},
            {"print_level": 0, "sb": "yes", "max_iter": 500},
        )
        sol = opti.solve()
        self.assertTrue(opti.stats()["success"])

        q_sol = np.asarray(sol.value(qs)).ravel()
        v_sol = np.asarray(sol.value(vs)).ravel()
        tau_sol = np.atleast_1d(np.asarray(sol.value(taus)).ravel())

        # Tolerances here are the solver's
        tolerance = 1e-6
        self.assertApprox(q_sol[0], q_init, tolerance)
        self.assertApprox(q_sol[-1], q_goal, tolerance)
        self.assertApprox(v_sol[-1], 0.0, tolerance)

        # Replay the optimal torques on the double precision model.
        q, v = np.array([q_init]), np.array([0.0])
        for i in range(nodes):
            a = pin.aba(self.model, self.data, q, v, np.array([tau_sol[i]]))
            v = v + dt * a
            q = pin.integrate(self.model, q, dt * v)
            self.assertApprox(q[0], q_sol[i + 1], tolerance)
            self.assertApprox(v[0], v_sol[i + 1], tolerance)


if __name__ == "__main__":
    unittest.main()
