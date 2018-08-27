import pinocchio as pin
from pinocchio.romeo_wrapper import RomeoWrapper

DISPLAY = False

## Load Romeo with RomeoWrapper
import os
current_path = os.getcwd()

# The model of Romeo is contained in the path PINOCCHIO_GIT_REPOSITORY/models/romeo
model_path = current_path + "/" + "../../models/romeo"
mesh_dir = model_path
urdf_filename = "romeo_small.urdf"
urdf_model_path = model_path + "/romeo_description/urdf/" + urdf_filename

robot = RomeoWrapper(urdf_model_path, [mesh_dir])

## alias
model = robot.model
data = robot.data

## do whatever, e.g. compute the center of mass position in the world frame
q0 = robot.q0
com = robot.com(q0)
# This last command is similar to
com2 = pin.centerOfMass(model,data,q0)

## load model into gepetto-gui
if DISPLAY:
  import gepetto.corbaserver

  cl = gepetto.corbaserver.Client()
  gui = cl.gui
  if gui.nodeExists("world"):
    gui.deleteNode("world","ON")

  robot.initDisplay(loadModel=False)
  robot.loadDisplayModel("pinocchio")
  robot.display(robot.q0)
  gui = robot.viewer.gui
