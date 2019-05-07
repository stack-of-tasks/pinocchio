##
## Copyright (c) 2018 CNRS
##
## This file is part of Pinocchio
## Pinocchio is free software: you can redistribute it
## and/or modify it under the terms of the GNU Lesser General Public
## License as published by the Free Software Foundation, either version
## 3 of the License, or (at your option) any later version.
##
## Pinocchio is distributed in the hope that it will be
## useful, but WITHOUT ANY WARRANTY; without even the implied warranty
## of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
## General Lesser Public License for more details. You should have
## received a copy of the GNU Lesser General Public License along with
## Pinocchio If not, see
## <http:##www.gnu.org/licenses/>.
##

##
## In this short script, we show how to a load a robot model from its URDF description.
## Here, the robot model is ROMEO, but a similar approach can be done by with rospkg.
##

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.display import *

DISPLAY = None
# DISPLAY = GepettoDisplay
# DISPLAY = MeshcatDisplay

## Load Romeo with RomeoWrapper
import os
current_path = os.getcwd()

# The model of Romeo is contained in the path PINOCCHIO_GIT_REPOSITORY/models/romeo
model_path = current_path + "/" + "../../models/romeo"
mesh_dir = model_path
urdf_filename = "romeo_small.urdf"
urdf_model_path = model_path + "/romeo_description/urdf/" + urdf_filename

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
    robot.setDisplay(DISPLAY())
    robot.initDisplay()
    if DISPLAY == GepettoDisplay:
        robot.loadDisplayModel("pinocchio")
    elif DISPLAY == MeshcatDisplay:
        pin.scaleGeometryModel(robot.visual_model,0.01)
        robot.viewer.open()
        robot.loadDisplayModel("pinocchio",color=[0.,0.,0.,1.])
    else:
        raise Exception("Unknown display")
    robot.display(q0)

raw_input("Press enter to exit.")
