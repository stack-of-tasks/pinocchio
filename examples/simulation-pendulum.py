# Parse input arguments
import argparse
import math
import sys
import time

import hppfcl as fcl
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer as Visualizer

parser = argparse.ArgumentParser()
parser.add_argument(
    "--with-cart",
    help="Add a cart at the base of the pendulum to simulate a cart pole system.",
    action="store_true",
)
parser.add_argument(
    "-N", help="Number of pendulums compositing the dynamical system.", type=int
)
args = parser.parse_args()

if args.N:
    N = args.N
else:
    N = 1  # number of pendulums

model = pin.Model()
geom_model = pin.GeometryModel()

parent_id = 0

if args.with_cart:
    cart_radius = 0.1
    cart_length = 5 * cart_radius
    cart_mass = 2.0
    joint_name = "joint_cart"

    geometry_placement = pin.SE3.Identity()
    geometry_placement.rotation = pin.Quaternion(
        np.array([0.0, 0.0, 1.0]), np.array([0.0, 1.0, 0.0])
    ).toRotationMatrix()

    joint_id = model.addJoint(
        parent_id, pin.JointModelPY(), pin.SE3.Identity(), joint_name
    )

    body_inertia = pin.Inertia.FromCylinder(cart_mass, cart_radius, cart_length)
    body_placement = geometry_placement
    model.appendBodyToJoint(
        joint_id, body_inertia, body_placement
    )  # We need to rotate the inertia as it is expressed in the LOCAL frame of the geometry

    shape_cart = fcl.Cylinder(cart_radius, cart_length)

    geom_cart = pin.GeometryObject(
        "shape_cart", joint_id, geometry_placement, shape_cart
    )
    geom_cart.meshColor = np.array([1.0, 0.1, 0.1, 1.0])
    geom_model.addGeometryObject(geom_cart)

    parent_id = joint_id
else:
    base_radius = 0.2
    shape_base = fcl.Sphere(base_radius)
    geom_base = pin.GeometryObject("base", 0, pin.SE3.Identity(), shape_base)
    geom_base.meshColor = np.array([1.0, 0.1, 0.1, 1.0])
    geom_model.addGeometryObject(geom_base)

joint_placement = pin.SE3.Identity()
body_mass = 1.0
body_radius = 0.1

for k in range(N):
    joint_name = "joint_" + str(k + 1)
    joint_id = model.addJoint(
        parent_id, pin.JointModelRX(), joint_placement, joint_name
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


visual_model = geom_model
# Initialize the viewer.
try:
    viz = Visualizer(model, geom_model, visual_model)
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

if args.with_cart:
    model.lowerPositionLimit[0] = model.upperPositionLimit[0] = 0.0

data_sim = model.createData()

t = 0.0
q = pin.randomConfiguration(model)
v = np.zeros((model.nv))
tau_control = np.zeros((model.nv))
damping_value = 0.1
for k in range(N):
    tic = time.time()
    tau_control = -damping_value * v  # small damping
    a = pin.aba(model, data_sim, q, v, tau_control)  # Forward dynamics

    # Semi-explicit integration
    v += a * dt
    q = pin.integrate(model, q, v * dt)  # Configuration integration

    viz.display(q)
    toc = time.time()
    ellapsed = toc - tic

    dt_sleep = max(0, dt - (ellapsed))
    time.sleep(dt_sleep)
    t += dt
