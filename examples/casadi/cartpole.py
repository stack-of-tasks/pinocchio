import sys

import casadi
import coal
import numpy as np
import pinocchio as pin
import pinocchio.casadi as cpin
from pinocchio.visualize import MeshcatVisualizer


def make_cartpole(ub=True):
    model = pin.Model()

    m1 = 1.0
    m2 = 0.1
    length = 0.5
    base_sizes = (0.4, 0.2, 0.05)

    base = pin.JointModelPX()
    base_id = model.addJoint(0, base, pin.SE3.Identity(), "base")

    if ub:
        pole = pin.JointModelRUBY()
    else:
        pole = pin.JointModelRY()
    pole_id = model.addJoint(1, pole, pin.SE3.Identity(), "pole")

    base_inertia = pin.Inertia.FromBox(m1, *base_sizes)
    pole_inertia = pin.Inertia(
        m2,
        np.array([0.0, 0.0, length / 2]),
        m2 / 5 * np.diagflat([1e-2, length**2, 1e-2]),
    )

    base_body_pl = pin.SE3.Identity()
    pole_body_pl = pin.SE3.Identity()
    pole_body_pl.translation = np.array([0.0, 0.0, length / 2])

    model.appendBodyToJoint(base_id, base_inertia, base_body_pl)
    model.appendBodyToJoint(pole_id, pole_inertia, pole_body_pl)

    # make visual/collision models
    collision_model = pin.GeometryModel()
    shape_base = coal.Box(*base_sizes)
    radius = 0.01
    shape_pole = coal.Capsule(radius, length)
    RED_COLOR = np.array([1, 0.0, 0.0, 1.0])
    WHITE_COLOR = np.array([1, 1.0, 1.0, 1.0])
    geom_base = pin.GeometryObject("link_base", base_id, shape_base, base_body_pl)
    geom_base.meshColor = WHITE_COLOR
    geom_pole = pin.GeometryObject("link_pole", pole_id, shape_pole, pole_body_pl)
    geom_pole.meshColor = RED_COLOR

    collision_model.addGeometryObject(geom_base)
    collision_model.addGeometryObject(geom_pole)
    visual_model = collision_model
    return model, collision_model, visual_model


class PinocchioCasadi:
    """Take a Pinocchio model, turn it into a Casadi model
    and define the appropriate graphs.
    """

    def __init__(self, model: pin.Model, timestep=0.05):
        self.model = model
        self.cmodel = cpin.Model(model)  # cast to CasADi model
        self.cdata = self.cmodel.createData()
        self.timestep = timestep
        self.create_dynamics()
        self.create_discrete_dynamics()

    def create_dynamics(self):
        """Create the acceleration expression and acceleration function."""
        nq = self.model.nq
        nu = 1
        nv = self.model.nv
        q = casadi.SX.sym("q", nq)
        v = casadi.SX.sym("v", nv)
        u = casadi.SX.sym("u", nu)
        dq_ = casadi.SX.sym("dq_", nv)
        self.u_node = u
        self.q_node = q
        self.v_node = v
        self.dq_ = dq_

        B = np.array([1, 0])
        tau = B @ u
        a = cpin.aba(self.cmodel, self.cdata, q, v, tau)
        self.acc = a
        self.acc_func = casadi.Function("acc", [q, v, u], [a], ["q", "v", "u"], ["a"])

    def create_discrete_dynamics(self):
        """
        Create the map `(q,v) -> (qnext, vnext)` using semi-implicit Euler integration.
        """
        q = self.q_node
        v = self.v_node
        u = self.u_node
        dq_ = self.dq_
        # q' = q + dq
        q_dq = cpin.integrate(self.cmodel, q, dq_)
        self.q_dq = q_dq
        # express acceleration using q' = q + dq
        a = self.acc_func(q_dq, v, u)

        dt = self.timestep
        vnext = v + a * dt
        qnext = cpin.integrate(self.cmodel, self.q_dq, dt * vnext)

        self.dyn_qv_fn_ = casadi.Function(
            "discrete_dyn",
            [q, dq_, v, u],
            [qnext, vnext],
            ["q", "dq_", "v", "u"],
            ["qnext", "vnext"],
        )

    def forward(self, x, u):
        nq = self.model.nq
        nv = self.model.nv
        q = x[:nq]
        v = x[nq:]
        dq_ = np.zeros(nv)
        qnext, vnext = self.dyn_qv_fn_(q, dq_, v, u)
        xnext = np.concatenate((qnext, vnext))
        return xnext

    def residual_fwd(self, x, u, xnext):
        nv = self.model.nv
        dq = np.zeros(nv)
        dqn = dq
        res = self.dyn_residual(x, u, xnext, dq, dqn)
        return res


class CartpoleDynamics(PinocchioCasadi):
    def __init__(self, timestep=0.05):
        model, collision_model, visual_model = make_cartpole()
        self.collision_model = collision_model
        self.visual_model = visual_model
        super().__init__(model=model, timestep=timestep)


dt = 0.02
cartpole = CartpoleDynamics(timestep=dt)
model = cartpole.model

print(model)

q0 = np.array([0.0, 0.95, 0.01])
q0 = pin.normalize(model, q0)
v = np.zeros(model.nv)
u = np.zeros(1)
a0 = cartpole.acc_func(q0, v, u)

print("a0:", a0)

x0 = np.append(q0, v)
xnext = cartpole.forward(x0, u)


def integrate_no_control(x0, nsteps):
    states_ = [x0.copy()]
    for t in range(nsteps):
        u = np.zeros(1)
        xnext = cartpole.forward(states_[t], u).ravel()
        states_.append(xnext)
    return states_


states_ = integrate_no_control(x0, nsteps=400)
states_ = np.stack(states_).T

try:
    viz = MeshcatVisualizer(
        model=model,
        collision_model=cartpole.collision_model,
        visual_model=cartpole.visual_model,
    )

    viz.initViewer()
    viz.loadViewerModel("pinocchio")

    qs_ = states_[: model.nq, :].T
    viz.play(q_trajectory=qs_, dt=dt)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
