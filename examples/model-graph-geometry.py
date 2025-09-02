import sys
import time

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# This example show how to:
#  - Construct a kinematics chain with ModelGraph API
#  - Add Geometries to a graph
#  - create a geometry model from a graph
#  - visualize it

g = pin.graph.ModelGraph()

# Adding bodies to the pin.graph.
# In pinocchio, since bodies are represented as frames,
# there's 2 ways to add a body to the graph :
#   - addBody, helper function
#   - addFrame, where you create the BodyFrame, yourself
g.addBody("body1", pin.Inertia.Identity())
g.addFrame("body2", pin.graph.BodyFrame(pin.Inertia.Identity()))
g.addFrame("body3", pin.graph.BodyFrame(pin.Inertia.Identity()))

# Adding a sensor to the pin.graph.
g.addFrame("sensor1", pin.graph.SensorFrame())

# Now we add the joints between every body/sensor we have in the pin.graph.
pose_b1_to_j1 = pin.SE3(np.eye(3), np.array([0.1, 0, 0]))  # pose of joint j1 wrt body1
r = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
pose_j1_to_b2 = pin.SE3(r, np.array([0.4, 0, 0]))  # pose of body2 wrt joint j1

# There are 2 ways to add joints to the graph.
# This helper function, where you specify the basics of a joint only.
g.addJoint(
    "j1",
    pin.graph.JointRevolute(np.array([0.0, 0.0, 1.0])),
    "body1",
    pose_b1_to_j1,
    "body2",
    pose_j1_to_b2,
)

# j2 will be biased by 50cm.

pose_b2_to_j2 = pin.SE3(
    np.eye(3), np.array([0.0, 0.2, -0.4])
)  # pose of joint j2 wrt body2
pose_j2_to_b3 = pin.SE3(np.eye(3), np.array([0, 0, 0]))  # pose of body3 wrt joint j2

# To add a joint with more details, such as limits, bias... the
# builder interface is necessary.
g.edgeBuilder().withName("j2").withJointType(
    pin.graph.JointPrismatic(np.array([0, 0, 1]))
).withSourceVertex("body2").withSourcePose(pose_b2_to_j2).withTargetVertex(
    "body3"
).withTargetPose(pose_j2_to_b3).build()

# Now we can choose which body will be our root its position, and build the model
kinematics_chain_from_body1 = pin.graph.buildModel(g, "body1", pin.SE3.Identity())
print("Kinematics chain from body1:")
print(kinematics_chain_from_body1)

# Now if you want to visualize, it's possible to add geometries to each body.
# Same as for joints, there is a builder interface, where all geometries
# parameters can be redefined.
# Type is to define in which geometry model the geometry will be added,
# it can be either visual, collision or both.
g.geometryBuilder().withBody("body1").withGeomType(pin.graph.GeomType.BOTH).withName(
    "body1_geom1"
).withPlacement(pin.SE3.Identity()).withGeom(
    pin.graph.Box(np.array([0.2, 0.2, 0.2]))
).withColor(np.array([0.3, 0.3, 0.7, 1])).build()
g.geometryBuilder().withBody("body2").withGeomType(pin.graph.GeomType.BOTH).withName(
    "body2_geom1"
).withPlacement(pin.SE3.Identity()).withGeom(
    pin.graph.Cylinder(np.array([0.1, 0.8]))
).withColor(np.array([0.7, 0.3, 0.3, 1])).build()
g.geometryBuilder().withBody("body3").withGeomType(pin.graph.GeomType.BOTH).withName(
    "body3_geom1"
).withPlacement(pin.SE3.Identity()).withGeom(pin.graph.Sphere(0.1)).build()

# And afterward build a pinocchio geometry model
visual_model = pin.graph.buildGeometryModel(g, kinematics_chain_from_body1, pin.VISUAL)
collision_model = pin.graph.buildGeometryModel(
    g, kinematics_chain_from_body1, pin.COLLISION
)

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in
# a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened separately by visiting the provided URL.
try:
    vizer = MeshcatVisualizer(
        kinematics_chain_from_body1, collision_model, visual_model
    )
    vizer.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

vizer.loadViewerModel()

q = np.array([0.3, 0.4])
vizer.display(q)
time.sleep(10.0)
