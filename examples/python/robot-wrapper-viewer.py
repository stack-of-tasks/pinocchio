##
## Copyright (c) 2018-2019 CNRS INRIA
##

##
## In this short script, we show how to a load a robot model from its URDF description.
## Here, the robot model is ROMEO, but a similar approach can be done by with rospkg.
##

import pinocchio as pin
pin.switchToNumpyMatrix()
from pinocchio.robot_wrapper import RobotWrapper
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

# Load the URDF model with RobotWrapper
# Conversion with str seems to be necessary when executing this file with ipython
current_path =  str(os.path.dirname(os.path.abspath(__file__)))
model_path = str(os.path.abspath(os.path.join(current_path, '../../models/romeo')))
mesh_dir = model_path
urdf_model_path = str(os.path.abspath(os.path.join(model_path, 'romeo_description/urdf/romeo_small.urdf')))

robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())

# alias
model = robot.model
data = robot.data

# do whatever, e.g. compute the center of mass position expressed in the world frame
q0 = robot.q0
com = robot.com(q0)

# This last command is similar to:
com2 = pin.centerOfMass(model,data,q0)

## load model into gepetto-gui
if VISUALIZER:
    robot.setVisualizer(VISUALIZER())
    robot.initViewer()
    if VISUALIZER == MeshcatVisualizer:
        robot.loadViewerModel("pinocchio", color=[0., 0., 0., 1.])
    else:
        robot.loadViewerModel("pinocchio")
    robot.display(q0)

input("Press enter to exit.")
