import time

import coal
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

# Forward dynamics of a ball tethered to a fixed anchor by a cable of constant length:
# a spherical pendulum, in 3D.
#
# The ball is carried by a translation joint (3 dof) and the cable is a single
# ConstantLengthConstraintModel between the anchor -- a point of the universe -- and the
# centre of the ball. The constraint keeps the ball on a sphere of radius
# `length_cable`, leaving 3 - 1 = 2 dof, and gravity does the rest.
#
# The constrained forward dynamics is the KKT system
#
#     [ M  -J^T ] [   ddq  ]   [ tau - b                   ]
#     [ J    0  ] [ lambda ] = [ -gamma - Kd*phid - Kp*phi ]
#
# where phi, phid and gamma are exactly the three quantities calc() stores in the
# constraint data: the position, velocity and zero-acceleration errors. The last two
# terms are the Baumgarte correction that keeps the cable from drifting. The cable
# tension is -lambda, positive when the cable pulls.
#
# Note: pin.constraintDynamics and pin.lcaba are currently only exposed in Python for
# RigidConstraintModel, hence the explicit KKT assembly here.

length_cable = 0.8
radius_ball = 0.08
radius_marker = 0.025
mass = 1.0

RED_COLOR = np.array([0.95, 0.25, 0.15, 1.0])
BLUE_COLOR = np.array([0.2, 0.55, 0.95, 1.0])
GREY_COLOR = np.array([0.45, 0.45, 0.45, 1.0])
SPHERE_COLOR = np.array([0.6, 0.6, 0.6, 0.15])
CABLE_COLOR = (240, 190, 40)

model = pin.Model()
model.name = "spherical-pendulum"
visual_model = pin.GeometryModel()

# The anchor: a point of the universe, at the origin.
placement_anchor = pin.SE3.Identity()
geom_obj = pin.GeometryObject("anchor", 0, placement_anchor, coal.Sphere(radius_marker))
geom_obj.meshColor = GREY_COLOR
visual_model.addGeometryObject(geom_obj)

# The sphere the ball is confined to, i.e. the constraint manifold itself.
geom_obj = pin.GeometryObject(
    "cable_sphere", 0, pin.SE3.Identity(), coal.Sphere(length_cable)
)
geom_obj.meshColor = SPHERE_COLOR
visual_model.addGeometryObject(geom_obj)

# The ball: a translation joint carries a point mass, no rotation involved.
ball_id = model.addJoint(0, pin.JointModelTranslation(), pin.SE3.Identity(), "ball")
model.appendBodyToJoint(
    ball_id, pin.Inertia.FromSphere(mass, radius_ball), pin.SE3.Identity()
)

geom_obj = pin.GeometryObject(
    "ball", ball_id, pin.SE3.Identity(), coal.Sphere(radius_ball)
)
geom_obj.meshColor = BLUE_COLOR
visual_model.addGeometryObject(geom_obj)

data = model.createData()

# Set the constraint: the anchor belongs to the universe (joint 0), the other point is
# the centre of the ball.
placement_ball = pin.SE3.Identity()
constraint_model = pin.ConstantLengthConstraintModel(
    model, 0, placement_anchor, ball_id, placement_ball, length_cable
)
constraint_model.name = "cable"
constraint_model.baumgarte_corrector_parameters.Kp = 1e4
constraint_model.baumgarte_corrector_parameters.Kd = 2.0 * np.sqrt(1e4)
constraint_data = constraint_model.createData()

Kp = constraint_model.baumgarte_corrector_parameters.Kp
Kd = constraint_model.baumgarte_corrector_parameters.Kd

print(f"model      : nq={model.nq}, nv={model.nv}")
print(f"constraint : {constraint_model.shortname()} '{constraint_model.name}'")
print(f"             residualSize={constraint_model.residualSize()}", end="")
print(f", length={constraint_model.getLength()}")
print(f"mechanism  : {model.nv} dof - {constraint_model.residualSize()} constraint")
print(f"baumgarte  : Kp={Kp:.0f}, Kd={Kd:.1f}\n")

# Release the ball 60 deg away from the vertical with a sideways kick. The ball starts
# exactly on the sphere and the kick is tangential, so both phi and phid vanish at t=0.
angle0 = np.deg2rad(60.0)
direction0 = np.array([np.sin(angle0), 0.0, -np.cos(angle0)])
speed0 = 1.2

q = length_cable * direction0
v = np.array([0.0, speed0, 0.0])
tau = np.zeros(model.nv)

# At t=0 the tension balances the radial acceleration: m (g cos(angle0) + v^2 / L).
gravity = abs(model.gravity.linear[2])
tension_ref = mass * (gravity * np.cos(angle0) + speed0**2 / length_cable)

dt = 1e-3
n_steps = 12000
n_display = 16  # keep one configuration out of n_display for the animation

configurations = []
worst_violation = 0.0

pin.forwardKinematics(model, data, q, v)
energy0 = pin.computeKineticEnergy(model, data, q, v) + pin.computePotentialEnergy(
    model, data, q
)

print("time [s]   |phi| [m]   tension [N]   energy [J]")
for k in range(n_steps):
    # a = 0 in the forward kinematics, so constraint_acceleration_error is exactly the
    # drift term gamma: the part of the constraint acceleration that does not depend on
    # the joint acceleration.
    pin.forwardKinematics(model, data, q, v, np.zeros(model.nv))
    pin.computeJointJacobians(model, data, q)
    constraint_model.calc(model, data, constraint_data)

    phi = constraint_data.constraint_position_error[0]
    phid = constraint_data.constraint_velocity_error[0]
    gamma = constraint_data.constraint_acceleration_error[0]

    # A scalar constraint: eigenpy returns the Jacobian as a 1-D array of size nv.
    J = constraint_model.jacobian(model, data, constraint_data)

    kkt = np.zeros((model.nv + 1, model.nv + 1))
    kkt[: model.nv, : model.nv] = pin.crba(model, data, q)
    kkt[: model.nv, model.nv] = -J
    kkt[model.nv, : model.nv] = J
    rhs = np.concatenate(
        [tau - pin.nonLinearEffects(model, data, q, v), [-gamma - Kd * phid - Kp * phi]]
    )

    solution = np.linalg.solve(kkt, rhs)
    a = solution[: model.nv]
    tension = -solution[model.nv]

    if k == 0:
        print(f"    t=0    tension {tension:.3f} N, analytic {tension_ref:.3f} N")

    # Semi-implicit Euler.
    v = v + a * dt
    q = pin.integrate(model, q, v * dt)

    worst_violation = max(worst_violation, abs(phi))
    if k % n_display == 0:
        configurations.append(q.copy())

    if k % (n_steps // 6) == 0:
        energy = pin.computeKineticEnergy(model, data, q, v)
        energy += pin.computePotentialEnergy(model, data, q)
        print(f"{k * dt:8.2f}   {abs(phi):9.2e}   {tension:11.3f}   {energy:10.4f}")

pin.forwardKinematics(model, data, q, v)
energy = pin.computeKineticEnergy(model, data, q, v) + pin.computePotentialEnergy(
    model, data, q
)

print(f"\nworst cable violation over {n_steps * dt:.0f} s : {worst_violation:.3e} m")
print(f"energy drift (semi-implicit Euler): {energy - energy0:+.3e} J")
assert worst_violation < 1e-3, "the cable drifted away"

# Display the mechanism. `open=True` would block until a browser client connects, so we
# print the URL instead.
viz = ViserVisualizer(model, visual_model, visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()
viz.display(configurations[0])

print(f"\nViser: http://{viz.viewer.get_host()}:{viz.viewer.get_port()}")
print("Grey sphere: the constraint manifold. The ball never leaves it.")

for q in configurations:
    viz.display(q)
    # The cable is not a body of the model, so it is drawn straight from the two
    # constrained points. Viser handles are immutable: re-adding under the same name
    # replaces the node.
    p_anchor = placement_anchor.translation
    p_ball = (viz.data.oMi[ball_id] * placement_ball).translation
    viz.viewer.scene.add_line_segments(
        "cable",
        points=np.stack([p_anchor, p_ball])[None].astype(np.float32),
        colors=CABLE_COLOR,
        thickness=0.012,
    )
    time.sleep(dt * n_display)
