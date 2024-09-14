import math
import sys
from os.path import abspath, dirname, join

import hppfcl as fcl
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer as Visualizer

# load model from example-robot urdf
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models/")

model_path = join(pinocchio_model_dir, "example-robot-data/robots")
mesh_dir = pinocchio_model_dir
urdf_filename = "ur5_robot.urdf"
urdf_model_path = join(join(model_path, "ur_description/urdf/"), urdf_filename)

model1, collision_model1, visual_model1 = pin.buildModelsFromUrdf(
    urdf_model_path, package_dirs=mesh_dir
)

# build model from scratch
model2 = pin.Model()
model2.name = "pendulum"
geom_model = pin.GeometryModel()

parent_id = 0
joint_placement = pin.SE3.Identity()
body_mass = 1.0
body_radius = 1e-2

joint_name = "joint_spherical"
joint_id = model2.addJoint(
    parent_id, pin.JointModelSpherical(), joint_placement, joint_name
)

body_inertia = pin.Inertia.FromSphere(body_mass, body_radius)
body_placement = joint_placement.copy()
body_placement.translation[2] = 0.1
model2.appendBodyToJoint(joint_id, body_inertia, body_placement)

geom1_name = "ball"
shape1 = fcl.Sphere(body_radius)
geom1_obj = pin.GeometryObject(geom1_name, joint_id, body_placement, shape1)
geom1_obj.meshColor = np.ones(4)
geom_model.addGeometryObject(geom1_obj)

geom2_name = "bar"
shape2 = fcl.Cylinder(body_radius / 4.0, body_placement.translation[2])
shape2_placement = body_placement.copy()
shape2_placement.translation[2] /= 2.0

geom2_obj = pin.GeometryObject(geom2_name, joint_id, shape2_placement, shape2)
geom2_obj.meshColor = np.array([0.0, 0.0, 0.0, 1.0])
geom_model.addGeometryObject(geom2_obj)

visual_model2 = geom_model

# join the two models, append pendulum to end effector
frame_id_end_effector = model1.getFrameId("tool0")
model, visual_model = pin.appendModel(
    model1,
    model2,
    visual_model1,
    visual_model2,
    frame_id_end_effector,
    pin.SE3.Identity(),
)

print(
    f"Check the joints of the appended model:\n {model} \n "
    "->Notice the spherical joint at the end."
)

try:
    viz = Visualizer(model, visual_model, visual_model)
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

# Load the robot in the viewer.
viz.loadViewerModel()

# Display a random robot configuration.
model.lowerPositionLimit.fill(-math.pi / 2)
model.upperPositionLimit.fill(+math.pi / 2)
q = pin.randomConfiguration(model)
viz.display(q)
