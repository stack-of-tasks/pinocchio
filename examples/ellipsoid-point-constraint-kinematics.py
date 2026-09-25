import sys
import time

import coal
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Constrained forward kinematics with EllipsoidPointConstraintModel: a point held on the
# surface of an ellipsoid.

# The plate is carried by a free-flyer, so 6 dof - 1 constraint = 5 dof. Each
# configuration is projected back onto it with a Newton iteration on the residual.
# The residual is homogeneous to a length, so its value is directly readable in metres.

radii = np.array([0.082998, 0.199991, 0.083001])
placement_ellipsoid = pin.SE3(pin.rpy.rpyToMatrix(0.0, -1.07, 0.0), np.zeros(3))

half_plate = 0.05
thickness = 0.006
mass = 0.5

RED_COLOR = np.array([0.95, 0.25, 0.15, 1.0])
BLUE_COLOR = np.array([0.2, 0.55, 0.95, 1.0])
ELLIPSOID_COLOR = 0x999999
ELLIPSOID_OPACITY = 0.25

model = pin.Model()
model.name = "plate-on-ellipsoid"
visual_model = pin.GeometryModel()

# The plate, and the material point of the plate that has to stay on the ellipsoid.
plate_id = model.addJoint(0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "plate")
model.appendBodyToJoint(
    plate_id,
    pin.Inertia.FromBox(mass, 2 * half_plate, 2 * half_plate, thickness),
    pin.SE3.Identity(),
)

geom_obj = pin.GeometryObject(
    "plate",
    plate_id,
    pin.SE3.Identity(),
    coal.Box(2 * half_plate, 2 * half_plate, thickness),
)
geom_obj.meshColor = BLUE_COLOR
visual_model.addGeometryObject(geom_obj)

# The contact point sits on the underside of the plate.
placement_contact = pin.SE3.Identity()
placement_contact.translation = -pin.ZAxis * thickness / 2.0
geom_obj = pin.GeometryObject(
    "contact_point", plate_id, placement_contact, coal.Sphere(0.008)
)
geom_obj.meshColor = RED_COLOR
visual_model.addGeometryObject(geom_obj)

data = model.createData()

# Set the constraint: the ellipsoid is carried by the universe (joint 0) at
# placement_ellipsoid, the material point by the plate at placement_contact.
constraint_model = pin.EllipsoidPointConstraintModel(
    model, 0, placement_ellipsoid, plate_id, placement_contact, radii
)
constraint_model.name = "point-on-ellipsoid"
constraint_data = constraint_model.createData()

print(f"model      : nq={model.nq}, nv={model.nv}")
print(f"constraint : {constraint_model.shortname()} '{constraint_model.name}'")
print(f"             residualSize={constraint_model.residualSize()}", end="")
print(f", radii={np.array2string(constraint_model.getRadii(), precision=6)}")
print(f"mechanism  : {model.nv} dof - {constraint_model.residualSize()} constraint\n")

eps = 1e-12
max_it = 50
n_steps = 240

# Drag the plate along a path that does not follow the surface: a circle of radius
# `drag_radius` around the axis of the ellipsoid, well outside it.
drag_radius = 0.30

q = pin.neutral(model)
configurations = []
worst_residual = 0.0

print("angle [deg]   pull [m]   Newton it   |residual| [m]")
for angle in np.linspace(0.0, 2.0 * np.pi, n_steps, endpoint=False):
    q[0] = drag_radius * np.cos(angle)
    q[1] = 0.25 * np.sin(2.0 * angle)
    q[2] = drag_radius * np.sin(angle)

    pin.forwardKinematics(model, data, q)
    constraint_model.calc(model, data, constraint_data)
    pull = constraint_data.constraint_position_error[0]

    for it in range(max_it):
        # calc() needs forwardKinematics, jacobian() needs computeJointJacobians on top.
        pin.forwardKinematics(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
        pin.computeJointJacobians(model, data, q)
        constraint_model.calc(model, data, constraint_data)

        residual = constraint_data.constraint_position_error[0]
        if abs(residual) < eps:
            break

        # A scalar constraint: eigenpy returns the Jacobian as a 1-D array of size nv,
        # and the least-norm correction of a single row is dq = -phi J^T / (J J^T).
        J = constraint_model.jacobian(model, data, constraint_data)
        q = pin.integrate(model, q, -residual * J / J.dot(J))

    configurations.append(q.copy())
    worst_residual = max(worst_residual, abs(residual))

    if len(configurations) % (n_steps // 6) == 1:
        print(
            f"{np.rad2deg(angle):9.1f}   {pull:8.3f}   {it:9d}   {abs(residual):14.2e}"
        )

print(f"\nworst residual over the sweep: {worst_residual:.3e} m")

# On the surface the algebraic residual x^T A x - 1 vanishes too, and the gradient of
# the metric residual is a unit vector: the residual really is a first order distance.
print(f"algebraic residual x^T A x - 1 : {constraint_data.algebraic_error:.3e}")
print(
    f"||grad phi|| on the surface    : {np.linalg.norm(constraint_data.gradient):.6f}"
)

# Display the mechanism.
try:
    import meshcat.geometry as mg

    viz = MeshcatVisualizer(model, visual_model, visual_model)
    viz.initViewer(open=True)
except ImportError as error:
    print(error)
    sys.exit(0)

viz.loadViewerModel()
viz.display(configurations[0])

# The ellipsoid belongs to the constraint not the Model.
viz.viewer["thorax"].set_object(
    mg.Ellipsoid(radii),
    mg.MeshLambertMaterial(
        color=ELLIPSOID_COLOR, transparent=True, opacity=ELLIPSOID_OPACITY
    ),
)
viz.viewer["thorax"].set_transform(placement_ellipsoid.homogeneous)

print("Grey ellipsoid: the constraint manifold. The red point never leaves it.")

dt = 6.0 / 60.0
for q in configurations:
    viz.display(q)
    time.sleep(dt)
