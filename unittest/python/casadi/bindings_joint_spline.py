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

PARABOLA_CONTROL_FRAMES_SX = [
    SX.sym(f"translation_xy_{i}", 2) for i, _ in enumerate(PARABOLA_CONTROL_FRAMES)
]


MIN_Q, MAX_Q = -1.0, 1.0


def build_cmodel(degree=3, control_frames=PARABOLA_CONTROL_FRAMES_SX):
    """Single spline joint carrying a unit box, plus the joint model itself."""

    frames = [
        cpin.SE3(SX.eye(3), casadi.vertcat(0.0, sx[0], sx[1])) for sx in control_frames
    ]
    joint = (
        cpin.JointModelSplineBuilder()
        .withDegree(degree)
        .withControlFrameVector(frames)
        .withOpenUniformKnots(MIN_Q, MAX_Q)
        .build()
    )
    model = cpin.Model()
    joint_id = model.addJoint(0, joint, cpin.SE3.Identity(), "spline")
    model.appendBodyToJoint(
        joint_id, cpin.Inertia.FromBox(1.0, 1.0, 1.0, 1.0), cpin.SE3.Identity()
    )
    return model, joint


class TestJointSplineCasadi(TestCase):
    def setUp(self):
        self.model, self.joint = build_cmodel()
        self.data = self.model.createData()

        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()

        self.cq = SX.sym("q", self.cmodel.nq)
        self.cv = SX.sym("v", self.cmodel.nv)
        self.ctau = SX.sym("tau", self.cmodel.nv)

    def test_optimize_control_vector(self):
        """Optimize the control vector to fit the parabola shape."""

        x = casadi.vertcat(*PARABOLA_CONTROL_FRAMES_SX)

        q_samples = np.linspace(MIN_Q, MAX_Q, 41)

        residuals = []
        for q_value in q_samples:
            q = SX([q_value])

            cpin.forwardKinematics(self.cmodel, self.cdata, q)

            # Translation of the spline joint in the world frame.
            translation = self.cdata.oMi[1].translation

            # Its translation should follow y = q and z = q**2.
            target = SX([0.0, q_value, q_value**2])
            residuals.append(translation - target)

        residual = casadi.vertcat(*residuals)
        cost = residual.T @ residual  # L2 norm of the residuals

        # PARABOLA_CONTROL_FRAMES as the initial guess.
        x0 = np.asarray(PARABOLA_CONTROL_FRAMES, dtype=float).reshape(-1)

        solver = casadi.nlpsol(
            "solver",
            "ipopt",
            {
                "x": x,
                "f": cost,
            },
            {
                "ipopt.print_level": 0,
                "print_time": False,
            },
        )

        result = solver(x0=x0)

        self.assertTrue(bool(solver.stats()["success"]))

        optimized = np.asarray(result["x"]).reshape(-1, 2)
        expected = np.asarray(PARABOLA_CONTROL_FRAMES)

        tolerance = 1e-6

        np.testing.assert_allclose(
            optimized,
            expected,
            atol=tolerance,
            rtol=tolerance,
        )

        gradient = casadi.gradient(cost, x)
        gradient_fun = casadi.Function("gradient", [x], [gradient])

        grad = np.asarray(gradient_fun(result["x"])).reshape(-1)

        np.testing.assert_allclose(
            grad,
            np.zeros_like(grad),
            atol=tolerance,
            rtol=tolerance,
        )


if __name__ == "__main__":
    unittest.main()
