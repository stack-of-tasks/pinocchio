# NOTE: this example needs RViz to be installed
# usage: start ROS master (roscore) and then run this test

from os.path import abspath, dirname, join

import pinocchio as pin
from pinocchio.visualize import RVizVisualizer

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

model_path = join(pinocchio_model_dir, "example-robot-data/robots")
mesh_dir = pinocchio_model_dir
urdf_filename = "talos_reduced.urdf"
urdf_model_path = join(join(model_path, "talos_data/robots"), urdf_filename)

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
viz = RVizVisualizer(model, collision_model, visual_model)

# Initialize the viewer.
viz.initViewer()
viz.loadViewerModel("pinocchio")

# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)

# Display another robot.
viz2 = RVizVisualizer(model, collision_model, visual_model)
viz2.initViewer(viz.viewer)
viz2.loadViewerModel(rootNodeName="pinocchio2")
q = q0.copy()
q[1] = 1.0
viz2.display(q)

input("Press enter to exit...")

viz.clean()
