##
## Copyright (c) 2019 CNRS INRIA
##

# This examples shows how to load and move a robot in meshcat.
# Note: this feature requires Meshcat to be installed, this can be done using
# pip install --user meshcat

import pinocchio as pin
pin.switchToNumpyMatrix()
import numpy as np
import os

try:
    # Python 2
    input = raw_input
except NameError:
    pass

from pinocchio.visualize import MeshcatVisualizer

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
current_path =  str(os.path.dirname(os.path.abspath(__file__)))
model_path = str(os.path.abspath(os.path.join(current_path, '../../models/romeo')))
mesh_dir = model_path
urdf_model_path = str(os.path.abspath(os.path.join(model_path, 'romeo_description/urdf/romeo_small.urdf')))

model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())
display = MeshcatVisualizer(model, collision_model, visual_model)
# pin.setGeometryMeshScales(visual_model,0.01)

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
display.initViewer(open=True)

input("Press enter to continue")

# Load the robot in the viewer.
# Color is needed here because the Romeo URDF doesn't contain any color, so the default color results in an
# invisible robot (alpha value set to 0).
display.loadViewerModel(color = [0.0, 0.0, 0.0, 1.0])

# Display a robot configuration.
q0 = np.matrix([
    0, 0, 0.840252, 0, 0, 0, 1,  # Free flyer
    0, 0, -0.3490658, 0.6981317, -0.3490658, 0,  # left leg
    0, 0, -0.3490658, 0.6981317, -0.3490658, 0,  # right leg
    0,  # chest
    1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2,  # left arm
    0, 0, 0, 0,  # head
    1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2,  # right arm
]).T
display.display(q0)

input("Displaying a single robot configuration. Press enter to continue")

# Display another robot.
red_robot_display = MeshcatVisualizer(model, collision_model, visual_model)
red_robot_display.initViewer(display.viewer)
red_robot_display.loadViewerModel(rootNodeName = "red_robot", color = [1.0, 0.0, 0.0, 0.5])
q = q0.copy()
q[1] = 1.0
red_robot_display.display(q)
input("Displaying a second robot with color red, semi-transparent. Press enter to exit")
