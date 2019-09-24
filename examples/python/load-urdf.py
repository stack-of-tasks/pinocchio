##
## Copyright (c) 2018-2019 CNRS
##

##
## In this short script, we show how to a load a robot model from its URDF description.
## Here, the robot model is ROMEO, but a similar approach can be done by with rospkg.
##

import pinocchio as pin
pin.switchToNumpyMatrix()
from pinocchio.visualize import *
import os

try:
    # Python 2
    input = raw_input
except NameError:
    pass

VISUALIZER = None
# VISUALIZER = GepettoVisualizer
# VISUALIZER = MeshcatVisualizer

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython
current_path =  str(os.path.dirname(os.path.abspath(__file__)))
model_path = str(os.path.abspath(os.path.join(current_path, '../../models/romeo')))
mesh_dir = model_path
urdf_model_path = str(os.path.abspath(os.path.join(model_path, 'romeo_description/urdf/romeo_small.urdf')))

model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())

# In this example, we do not explicitely need collision data and visual data
data = model.createData()

# do whatever, e.g. compute the center of mass position expressed in the world frame
q0 = pin.neutral(model)
com = pin.centerOfMass(model,data,q0)

## load model into gepetto-gui
if VISUALIZER:
    display = VISUALIZER(model, collision_model, visual_model)
    display.initViewer()
    if VISUALIZER == MeshcatVisualizer:
        display.loadViewerModel("pinocchio", color=[0., 0., 0., 1.])
    else:
        display.loadViewerModel("pinocchio")
    display.display(q0)

input("Press enter to exit.")
