import pinocchio as pin
import hppfcl as fcl
import numpy as np
import math
import time
import sys

N = 10  # number of pendulums
model = pin.Model()
geom_model = pin.GeometryModel()

parent_id = 0
joint_placement = pin.SE3.Identity()
body_mass = 1.0
body_radius = 0.1

shape0 = fcl.Sphere(body_radius)
geom0_obj = pin.GeometryObject("base", 0, pin.SE3.Identity(), shape0)
geom0_obj.meshColor = np.array([1.0, 0.1, 0.1, 1.0])
geom_model.addGeometryObject(geom0_obj)

for k in range(N):
    joint_name = "joint_" + str(k + 1)
    joint_id = model.addJoint(
        parent_id, pin.JointModelRY(), joint_placement, joint_name
    )

    body_inertia = pin.Inertia.FromSphere(body_mass, body_radius)
    body_placement = joint_placement.copy()
    body_placement.translation[2] = 1.0
    model.appendBodyToJoint(joint_id, body_inertia, body_placement)

    geom1_name = "ball_" + str(k + 1)
    shape1 = fcl.Sphere(body_radius)
    geom1_obj = pin.GeometryObject(geom1_name, joint_id, body_placement, shape1)
    geom1_obj.meshColor = np.ones((4))
    geom_model.addGeometryObject(geom1_obj)

    geom2_name = "bar_" + str(k + 1)
    shape2 = fcl.Cylinder(body_radius / 4.0, body_placement.translation[2])
    shape2_placement = body_placement.copy()
    shape2_placement.translation[2] /= 2.0

    geom2_obj = pin.GeometryObject(geom2_name, joint_id, shape2_placement, shape2)
    geom2_obj.meshColor = np.array([0.0, 0.0, 0.0, 1.0])
    geom_model.addGeometryObject(geom2_obj)

    parent_id = joint_id
    joint_placement = body_placement.copy()

from pinocchio.visualize import GepettoVisualizer

visual_model = geom_model
viz = GepettoVisualizer(model, geom_model, visual_model)

# Initialize the viewer.
try:
    viz.initViewer()
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install gepetto-viewer"
    )
    print(err)
    sys.exit(0)

try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print(
        "Error while loading the viewer model. It seems you should start gepetto-viewer"
    )
    print(err)
    sys.exit(0)

# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)

# Play a bit with the simulation
dt = 0.01
T = 5

N = math.floor(T / dt)

model.lowerPositionLimit.fill(-math.pi)
model.upperPositionLimit.fill(+math.pi)
q = pin.randomConfiguration(model)
v = np.zeros((model.nv))

t = 0.0
data_sim = model.createData()
for k in range(N):
    tau_control = np.zeros((model.nv))
    a = pin.aba(model, data_sim, q, v, tau_control)  # Forward dynamics

    v += a * dt
    # q += v*dt
    q = pin.integrate(model, q, v * dt)

    viz.display(q)
    time.sleep(dt)
    t += dt
