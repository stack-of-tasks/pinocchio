import time

import coal
import numpy as np
import pinocchio as pin
import trimesh
from pinocchio.visualize import ViserVisualizer

# Constrained forward kinematics with EllipsoidPointConstraintModel: a point held on the
# surface of an ellipsoid.
#
# This is the scapulothoracic mechanism of the shoulder models of Naaim, Soodmand and
# Quental: a body -- here a plate standing for the scapula -- glides on a thoracic
# ellipsoid, and the contact is modelled by a single scalar equation stating that one
# material point of the plate belongs to the surface.
#
# The plate is carried by a free-flyer, so 6 dof - 1 constraint = 5 dof. Below the plate
# is dragged along a path that leaves the surface, and each configuration is projected
# back onto it with a Newton iteration on the residual. The residual is homogeneous to a
# length, so its value is directly readable in metres.

# Thoracic ellipsoid of the Seth shoulder model, in the order of the .osim file.
radii = np.array([0.082998, 0.199991, 0.083001])

# Pose of the ellipsoid in the world. The thorax is the universe here.
placement_ellipsoid = pin.SE3(pin.rpy.rpyToMatrix(0.0, -1.07, 0.0), np.zeros(3))

half_plate = 0.05
thickness = 0.006
mass = 0.5

RED_COLOR = np.array([0.95, 0.25, 0.15, 1.0])
BLUE_COLOR = np.array([0.2, 0.55, 0.95, 1.0])
ELLIPSOID_COLOR = np.array([0.6, 0.6, 0.6, 0.25])

model = pin.Model()
model.name = "scapula-on-thorax"
visual_model = pin.GeometryModel()

# The plate, and the material point of the plate that has to stay on the ellipsoid.
plate_id = model.addJoint(0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "scapula")
model.appendBodyToJoint(
    plate_id,
    pin.Inertia.FromBox(mass, 2 * half_plate, 2 * half_plate, thickness),
    pin.SE3.Identity(),
)

geom_obj = pin.GeometryObject(
    "scapula",
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
constraint_model.name = "scapulothoracic"
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
assert worst_residual < 1e-10, "the projection did not converge"

# On the surface the algebraic residual x^T A x - 1 vanishes too, and the gradient of
# the metric residual is a unit vector: the residual really is a first order distance.
print(f"algebraic residual x^T A x - 1 : {constraint_data.algebraic_error:.3e}")
print(
    f"||grad phi|| on the surface    : {np.linalg.norm(constraint_data.gradient):.6f}"
)

# Display the mechanism. `open=True` would block until a browser client connects, so we
# print the URL instead.
viz = ViserVisualizer(model, visual_model, visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()
viz.display(configurations[0])

# The ellipsoid is not a body of the model -- it belongs to the constraint -- and
# ViserVisualizer only handles Box, Sphere, Cylinder, Convex and meshes anyway. Draw it
# straight through the viser server, from a unit sphere scaled by the radii.
unit_sphere = trimesh.creation.icosphere(subdivisions=4, radius=1.0)
ellipsoid_vertices = np.asarray(unit_sphere.vertices) * radii
ellipsoid_vertices = ellipsoid_vertices @ placement_ellipsoid.rotation.T
ellipsoid_vertices += placement_ellipsoid.translation
viz.viewer.scene.add_mesh_simple(
    "thorax",
    ellipsoid_vertices,
    np.asarray(unit_sphere.faces),
    color=ELLIPSOID_COLOR[:3],
    opacity=ELLIPSOID_COLOR[3],
)

print(f"\nViser: http://{viz.viewer.get_host()}:{viz.viewer.get_port()}")
print("Grey ellipsoid: the constraint manifold. The red point never leaves it.")

dt = 6.0 / 60.0
for q in configurations:
    viz.display(q)
    time.sleep(dt)
