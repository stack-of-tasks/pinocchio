import sys
from pathlib import Path

import casadi
import numpy as np
import pinocchio as pin
import pinocchio.casadi as cpin

# This use the example-robot-data submodule, but if you have it already properly
# installed in your PYTHONPATH, there is no need for this sys.path thing
path = Path(__file__).parent.parent.parent / "models" / "example-robot-data" / "python"
sys.path.append(str(path))
import example_robot_data  # noqa: E402

# Problem parameters
x_goal = [1, 0, 1.5, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
x0 = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
nodes = 80
dt = 0.02

# Quadcopter parameters
d_cog, cf, cm = 0.1525, 6.6e-5, 1e-6

tau_f = np.array(
    [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0, 1.0],
        [0.0, d_cog, 0.0, -d_cog],
        [-d_cog, 0.0, d_cog, 0.0],
        [-cm / cf, cm / cf, -cm / cf, cm / cf],
    ]
)

# Other variables
x_nom = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]


def actuation_model():
    u = casadi.SX.sym("u", 4)  # rotor velocities
    tau = tau_f @ u

    return casadi.Function("act_model", [u], [tau], ["u"], ["tau"])


def state_integrate(model):
    q = casadi.SX.sym("dq", model.nq)
    dq = casadi.SX.sym("q", model.nv)
    v = casadi.SX.sym("v", model.nv)
    dv = casadi.SX.sym("dv", model.nv)

    q_next = cpin.integrate(model, q, dq)
    v_next = v + dv

    dx = casadi.vertcat(dq, dv)
    x = casadi.vertcat(q, v)
    x_next = casadi.vertcat(q_next, v_next)

    return casadi.Function("integrate", [x, dx], [x_next], ["x", "dx"], ["x_next"])


def state_difference(model):
    q0 = casadi.SX.sym("q0", model.nq)
    q1 = casadi.SX.sym("q1", model.nq)
    v0 = casadi.SX.sym("v0", model.nv)
    v1 = casadi.SX.sym("v1", model.nv)

    q_diff = cpin.difference(model, q0, q1)
    v_diff = v1 - v0

    x0 = casadi.vertcat(q0, v0)
    x1 = casadi.vertcat(q1, v1)
    x_diff = casadi.vertcat(q_diff, v_diff)

    return casadi.Function("difference", [x0, x1], [x_diff], ["x0", "x1"], ["x_diff"])


def euler_integration(model, data, dt):
    nu = 4
    u = casadi.SX.sym("u", nu)

    # tau = casadi.vertcat(np.zeros(model.nv - nu), u)
    tau = actuation_model()(u)

    q = casadi.SX.sym("q", model.nq)
    v = casadi.SX.sym("v", model.nv)

    a = cpin.aba(model, data, q, v, tau)

    dq = v * dt + a * dt**2
    dv = a * dt

    x = casadi.vertcat(q, v)
    dx = casadi.vertcat(dq, dv)
    x_next = state_integrate(model)(x, dx)

    return casadi.Function("int_dyn", [x, u], [x_next], ["x", "u"], ["x_next"])


def cost_quadratic_state_error(model):
    dx = casadi.SX.sym("dx", model.nv * 2)

    x_N = state_integrate(model)(x_nom, dx)
    e_goal = state_difference(model)(x_N, x_goal)

    cost = 0.5 * e_goal.T @ e_goal

    return casadi.Function("quad_cost", [dx], [cost], ["dx"], ["cost"])


class OptimalControlProblem:
    def __init__(self, model, terminal_soft_constraint=False):
        self.opti = casadi.Opti()

        self.model = model
        self.data = self.model.createData()

        self.c_model = cpin.Model(self.model)
        self.c_data = self.c_model.createData()

        nv = self.c_model.nv
        nu = 4

        self.c_dxs = self.opti.variable(2 * nv, nodes + 1)  # state trajectory
        self.c_us = self.opti.variable(nu, nodes)  # control trajectory

        # Objective function
        obj = 0

        # State & Control regularization
        for i in range(nodes):
            x_i = state_integrate(self.c_model)(x_nom, self.c_dxs[:, i])
            e_reg = state_difference(self.c_model)(x_nom, x_i)
            obj += (
                1e-5 * 0.5 * e_reg.T @ e_reg
                + 1e-5 * 0.5 * self.c_us[:, i].T @ self.c_us[:, i]
            )
        if terminal_soft_constraint:
            obj += 1000 * cost_quadratic_state_error(self.c_model)(self.c_dxs[:, nodes])

        self.opti.minimize(obj)

        # Dynamical constraints
        for i in range(nodes):
            x_i = state_integrate(self.c_model)(x_nom, self.c_dxs[:, i])
            x_i_1 = state_integrate(self.c_model)(x_nom, self.c_dxs[:, i + 1])
            f_x_u = euler_integration(self.c_model, self.c_data, dt)(
                x_i, self.c_us[:, i]
            )
            gap = state_difference(self.c_model)(f_x_u, x_i_1)

            self.opti.subject_to(gap == [0] * 12)

        # Control constraints
        self.opti.subject_to(self.opti.bounded(0, self.c_us, 5))

        # Final constraint
        if not terminal_soft_constraint:
            x_N = state_integrate(self.c_model)(x_nom, self.c_dxs[:, nodes])
            e_goal = state_difference(self.c_model)(x_N, x_goal)
            self.opti.subject_to(e_goal == [0] * 12)

        # Initial state
        x_0 = state_integrate(self.c_model)(x_nom, self.c_dxs[:, 0])
        self.opti.subject_to(state_difference(self.c_model)(x0, x_0) == [0] * 12)

        # Warm start
        self.opti.set_initial(
            self.c_dxs, np.vstack([np.zeros(12) for _ in range(nodes + 1)]).T
        )
        self.opti.set_initial(
            self.c_us, np.vstack([np.zeros(4) for _ in range(nodes)]).T
        )

    def solve(self, approx_hessian=True):
        opts = {"verbose": False}
        opts["ipopt"] = {
            "max_iter": 1000,
            "linear_solver": "mumps",
            "tol": 3.82e-6,
            "mu_strategy": "adaptive",
        }

        if approx_hessian:
            opts["ipopt"]["hessian_approximation"] = "limited-memory"

        # Solver initialization
        self.opti.solver("ipopt", opts)  # set numerical backend

        try:
            self.sol = self.opti.solve()
        except:  # noqa: E722
            self.sol = self.opti.debug

        self._retract_trajectory()
        self._compute_gaps()

    def _retract_trajectory(self):
        self.xs = []
        self.us = []
        self.gaps = []

        nq = self.model.nq
        nv = self.model.nv

        for idx, (dx_sol, u_sol) in enumerate(
            zip(self.sol.value(self.c_dxs).T, self.sol.value(self.c_us).T)
        ):
            q = pin.integrate(self.model, np.array(x_nom)[:nq], dx_sol[:nv])
            v = dx_sol[nv:]

            self.xs.append(np.concatenate([q, v]))
            self.us.append(u_sol)

        q = pin.integrate(
            self.model, np.array(x_nom)[:nq], self.sol.value(self.c_dxs).T[nodes, :nv]
        )
        v = self.sol.value(self.c_dxs).T[nodes, nv:]
        self.xs.append(np.concatenate([q, v]))

    def _compute_gaps(self):
        self.gaps = {"vector": [np.zeros(self.model.nv * 2)], "norm": [0]}

        nq = self.model.nq
        _nv = self.model.nv

        for idx, (x, u) in enumerate(zip(self.xs, self.us)):
            x_pin = self._simulate_step(x, u)

            gap_q = pin.difference(self.model, x_pin[:nq], self.xs[idx + 1][:nq])
            gap_v = self.xs[idx + 1][nq:] - x_pin[nq:]

            gap = np.concatenate([gap_q, gap_v])
            self.gaps["vector"].append(gap)
            self.gaps["norm"].append(np.linalg.norm(gap))

    def _simulate_step(self, x, u):
        nq = self.model.nq
        _nv = self.model.nv

        q = x[:nq]
        v = x[nq:]

        tau = tau_f @ u

        a = pin.aba(self.model, self.data, q, v, tau)

        dq = v * dt + a * dt**2
        dv = a * dt

        q_next = pin.integrate(self.model, q, dq)
        v_next = v + dv

        x_next = np.concatenate([q_next, v_next])

        return x_next


def main():
    robot = example_robot_data.load("hector")
    model = robot.model

    oc_problem = OptimalControlProblem(model, terminal_soft_constraint=False)

    oc_problem.solve(approx_hessian=True)

    # --------------PLOTS-----------
    try:
        import matplotlib.pyplot as plt

        _, axs0 = plt.subplots(nrows=2)

        xs = np.vstack(oc_problem.xs)
        axs0[0].plot(xs[:, :3])
        axs0[0].set_title("Quadcopter position")

        axs0[1].plot(oc_problem.gaps["norm"])
        axs0[1].set_title("Multiple shooting node gaps")

        _, axs1 = plt.subplots(nrows=4)
        us = np.vstack(oc_problem.us)

        for idx, ax in enumerate(axs1):
            ax.plot(us[:, idx])

        plt.show(block=False)
    except ImportError as err:
        print(
            "Error while initializing the viewer. "
            "It seems you should install Python meshcat"
        )
        print(err)
        sys.exit(0)


if __name__ == "__main__":
    main()
