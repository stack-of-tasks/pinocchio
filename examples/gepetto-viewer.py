# NOTE: this example needs gepetto-gui to be installed
# usage: launch gepetto-gui and then run this test

import pinocchio as pin
pin.switchToNumpyMatrix()
import numpy as np
import os

from pinocchio.visualize import GepettoVisualizer

try:
    # Python 2
    input = raw_input
except NameError:
    pass

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
current_path =  str(os.path.dirname(os.path.abspath(__file__)))
model_path = str(os.path.abspath(os.path.join(current_path, '../models/others/robots')))
mesh_dir = model_path
urdf_model_path = str(os.path.abspath(os.path.join(model_path, 'romeo_description/urdf/romeo_small.urdf')))

model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())
viz = GepettoVisualizer(model, collision_model, visual_model)

# Initialize the viewer.
viz.initViewer()
viz.loadViewerModel("pinocchio")

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
viz.display(q0)

input("Displaying a single robot configuration. Press enter to continue")

# Display another robot.
viz2 = GepettoVisualizer(model, collision_model, visual_model)
viz2.initViewer(viz.viewer)
viz2.loadViewerModel(rootNodeName = "pinocchio2")
q = q0.copy()
q[1] = 1.0
viz2.display(q)
input("Displaying a second robot. Press enter to exit")
