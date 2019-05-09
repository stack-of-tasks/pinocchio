##
## Copyright (c) 2018-2019 CNRS
##

##
## In this short script, we show how to a load a robot model from its URDF description.
## Here, the robot model is ROMEO, but a similar approach can be done by with rospkg.
##

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.display import *
import os

DISPLAY = None
# DISPLAY = GepettoDisplay
# DISPLAY = MeshcatDisplay

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
if DISPLAY:
    robot.setDisplay(DISPLAY(),copy_models = (DISPLAY == MeshcatDisplay))
    robot.initDisplay()
    if DISPLAY == GepettoDisplay:
        robot.loadDisplayModel("pinocchio")
    elif DISPLAY == MeshcatDisplay:
        pin.setGeometryMeshScales(robot.disp.visual_model,0.01)
        robot.viewer.open()
        robot.loadDisplayModel("pinocchio",color=[0.,0.,0.,1.])
    else:
        raise Exception("Unknown display")
    robot.display(q0)

raw_input("Press enter to exit.")
