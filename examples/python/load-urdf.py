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

model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())

# In this example, we do not explicitely need collision data and visual data
data = model.createData()

# do whatever, e.g. compute the center of mass position expressed in the world frame
q0 = pin.neutral(model)
com = pin.centerOfMass(model,data,q0)

## load model into gepetto-gui
if DISPLAY:
    display = DISPLAY(model, collision_model, visual_model)
    display.initDisplay()
    if DISPLAY == GepettoDisplay:
        display.loadDisplayModel("pinocchio")
    elif DISPLAY == MeshcatDisplay:
        pin.setGeometryMeshScales(visual_model,0.01)
        display.viewer.open()
        display.loadDisplayModel("pinocchio",color=[0.,0.,0.,1.])
    else:
        raise Exception("Unknown display")
    display.display(q0)

raw_input("Press enter to exit.")
