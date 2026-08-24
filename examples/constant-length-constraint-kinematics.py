import time

import coal
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

# Constrained forward kinematics of a four-bar linkage ,modelled as an open kinematic tree:
# a crank and a rocker, both pinned to the ground, and the coupler is replaced by a single scalar constraint:
# ConstantLengthConstraintModel keeps the distance between the crank tip and the rocker
# tip equal to the coupler length. 2 DoF - 1 constraint = 1 DoF: driving the crank determines the rocker.

length_ground = 1.0
length_crank = 0.35
length_coupler = 0.9 # not too small so the crank can perform a full revolution.
length_rocker = 0.8

mass = 1.0
width = 0.02
radius = 0.022

RED_COLOR = np.array([0.95, 0.25, 0.15, 1.0])
BLUE_COLOR = np.array([0.2, 0.55, 0.95, 1.0])
GREY_COLOR = np.array([0.45, 0.45, 0.45, 1.0])
COUPLER_COLOR = (240, 190, 40)

# coal cylinders are aligned with Z and centred, the links point along X.
# (ViserVisualizer supports Box, Sphere, Cylinder, Convex and meshes -- not Capsule.)
rotation_z_to_x = pin.Quaternion.FromTwoVectors(pin.ZAxis, pin.XAxis).matrix()

placement_center_crank = pin.SE3.Identity()
placement_center_crank.translation = pin.XAxis * length_crank / 2.0
placement_shape_crank = placement_center_crank.copy()
placement_shape_crank.rotation = rotation_z_to_x

placement_center_rocker = pin.SE3.Identity()
placement_center_rocker.translation = pin.XAxis * length_rocker / 2.0
placement_shape_rocker = placement_center_rocker.copy()
placement_shape_rocker.rotation = rotation_z_to_x

placement_shape_ground = pin.SE3.Identity()
placement_shape_ground.translation = pin.XAxis * length_ground / 2.0
placement_shape_ground.rotation = rotation_z_to_x

# The two points tied by the constraint, at the tip of each link.
placement_crank_tip = pin.SE3.Identity()
placement_crank_tip.translation = pin.XAxis * length_crank
placement_rocker_tip = pin.SE3.Identity()
placement_rocker_tip.translation = pin.XAxis * length_rocker

# Build the open kinematic tree: two revolute joints pinned to the ground.
model = pin.Model()
model.name = "four-bar"
visual_model = pin.GeometryModel()

geom_obj = pin.GeometryObject(
    "ground", 0, placement_shape_ground, coal.Cylinder(0.012, length_ground)
)
geom_obj.meshColor = GREY_COLOR
visual_model.addGeometryObject(geom_obj)

crank_id = model.addJoint(0, pin.JointModelRZ(), pin.SE3.Identity(), "crank")
model.appendBodyToJoint(
    crank_id,
    pin.Inertia.FromBox(mass, length_crank, width, width),
    placement_center_crank,
)
geom_obj = pin.GeometryObject(
    "crank", crank_id, placement_shape_crank, coal.Cylinder(radius, length_crank)
)
geom_obj.meshColor = BLUE_COLOR
visual_model.addGeometryObject(geom_obj)

rocker_placement = pin.SE3.Identity()
rocker_placement.translation = pin.XAxis * length_ground
rocker_id = model.addJoint(0, pin.JointModelRZ(), rocker_placement, "rocker")
model.appendBodyToJoint(
    rocker_id,
    pin.Inertia.FromBox(mass, length_rocker, width, width),
    placement_center_rocker,
)
geom_obj = pin.GeometryObject(
    "rocker", rocker_id, placement_shape_rocker, coal.Cylinder(radius, length_rocker)
)
geom_obj.meshColor = BLUE_COLOR
visual_model.addGeometryObject(geom_obj)

geom_obj = pin.GeometryObject(
    "crank_tip", crank_id, placement_crank_tip, coal.Sphere(0.035)
)
geom_obj.meshColor = RED_COLOR
visual_model.addGeometryObject(geom_obj)

geom_obj = pin.GeometryObject(
    "rocker_tip", rocker_id, placement_rocker_tip, coal.Sphere(0.035)
)
geom_obj.meshColor = RED_COLOR
visual_model.addGeometryObject(geom_obj)

data = model.createData()

# Set the constraint that replaces the coupler.
constraint_model = pin.ConstantLengthConstraintModel(
    model,
    crank_id,
    placement_crank_tip,
    rocker_id,
    placement_rocker_tip,
    length_coupler,
)
constraint_model.name = "coupler"
constraint_data = constraint_model.createData()

print(f"model      : nq={model.nq}, nv={model.nv}")
print(f"constraint : {constraint_model.shortname()} '{constraint_model.name}'")
print(f"             residualSize={constraint_model.residualSize()}", end="")
print(f", length={constraint_model.getLength()}")
print(f"mechanism  : {model.nv} dof - {constraint_model.residualSize()} constraint\n")

# The crank is driven, the rocker is solved for.
crank_idx_v = model.joints[crank_id].idx_v
rocker_idx_v = model.joints[rocker_id].idx_v

eps = 1e-12
max_it = 50
n_steps = 240
zero = np.zeros(model.nv)

# Sweep the crank over a full revolution, warm-starting each solve with the previous
# solution so that the assembly branch of the mechanism stays the same.
q = pin.neutral(model)
q[rocker_idx_v] = 2.0  # pick the "elbow up" branch
configurations = []
worst_residual = 0.0

print("crank [deg]   rocker [deg]   Newton it   |residual| [m]")
for theta in np.linspace(0.0, 2.0 * np.pi, n_steps, endpoint=False):
    q[crank_idx_v] = theta

    for it in range(max_it):
        # calc() needs forwardKinematics, jacobian() needs computeJointJacobians on top.
        pin.forwardKinematics(model, data, q, zero, zero)
        pin.computeJointJacobians(model, data, q)
        constraint_model.calc(model, data, constraint_data)

        residual = constraint_data.constraint_position_error[0]
        if abs(residual) < eps:
            break

        # The constraint has a single row, so eigenpy hands the Jacobian back as a 1-D
        # array of shape (nv,) rather than the (1, nv) of a 3D constraint.
        J = constraint_model.jacobian(model, data, constraint_data)
        dq = np.zeros(model.nv)
        dq[rocker_idx_v] = -residual / J[rocker_idx_v]
        q = pin.integrate(model, q, dq)

    configurations.append(q.copy())
    worst_residual = max(worst_residual, abs(residual))

    if len(configurations) % (n_steps // 8) == 1:
        print(
            f"{np.rad2deg(theta):9.1f}   {np.rad2deg(q[rocker_idx_v]):12.1f}"
            f"   {it:9d}   {abs(residual):.2e}"
        )

print(f"\nworst residual over the sweep: {worst_residual:.3e} m")
assert worst_residual < 1e-10, "the loop did not close"

# A joint velocity is compatible with the mechanism iff it lies in the kernel of the
# constraint Jacobian: here, the rocker rate that follows a unit crank rate.
q = configurations[0]
pin.forwardKinematics(model, data, q, zero, zero)
pin.computeJointJacobians(model, data, q)
constraint_model.calc(model, data, constraint_data)
J = constraint_model.jacobian(model, data, constraint_data)

v = np.zeros(model.nv)
v[crank_idx_v] = 1.0
v[rocker_idx_v] = -J[crank_idx_v] / J[rocker_idx_v]
print(f"constraint Jacobian at theta=0: {np.array2string(J, precision=4)}")
print(f"admissible velocity           : {np.array2string(v, precision=4)}")
print(f"J @ v (should vanish)         : {J @ v:.2e}")

# Display the mechanism. `open=True` would block until a browser client connects, so we
# print the URL instead.
viz = ViserVisualizer(model, visual_model, visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()
viz.display(configurations[0])

print(f"\nViser: http://{viz.viewer.get_host()}:{viz.viewer.get_port()}")
print("Red spheres: the two constrained points. Yellow rod: the constraint itself.")

dt = 1.0 / 60.0
for q in configurations * 3:
    viz.display(q)
    # The coupler is not a body of the model, so it is drawn straight from the two
    # constrained points. Viser handles are immutable: re-adding under the same name
    # replaces the node.
    p1 = (viz.data.oMi[crank_id] * placement_crank_tip).translation
    p2 = (viz.data.oMi[rocker_id] * placement_rocker_tip).translation
    viz.viewer.scene.add_line_segments(
        "coupler",
        points=np.stack([p1, p2])[None].astype(np.float32),
        colors=COUPLER_COLOR,
        thickness=0.02,
    )
    time.sleep(dt)
