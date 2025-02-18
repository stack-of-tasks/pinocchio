#
# In this short script, we show how to use RobotWrapper
# integrating different kinds of viewers
#

from pathlib import Path
from sys import argv

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import GepettoVisualizer, MeshcatVisualizer

# If you want to visualize the robot in this example,
# you can choose which visualizer to employ
# by specifying an option from the command line:
# GepettoVisualizer: -g
# MeshcatVisualizer: -m
VISUALIZER = None
if len(argv) > 1:
    opt = argv[1]
    if opt == "-g":
        VISUALIZER = GepettoVisualizer
    elif opt == "-m":
        VISUALIZER = MeshcatVisualizer
    else:
        raise ValueError("Unrecognized option: " + opt)

# Load the URDF model with RobotWrapper
# Conversion with str seems to be necessary when executing this file with ipython
pinocchio_model_dir = Path(__file__).parent.parent / "models"

model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
urdf_filename = "talos_reduced.urdf"
urdf_model_path = model_path / "talos_data/robots" / urdf_filename

robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())

# alias
model = robot.model
data = robot.data

# do whatever, e.g. compute the center of mass position expressed in the world frame
q0 = robot.q0
com = robot.com(q0)

# This last command is similar to:
com2 = pin.centerOfMass(model, data, q0)

# Show model with a visualizer of your choice
if VISUALIZER:
    robot.setVisualizer(VISUALIZER())
    robot.initViewer()
    robot.loadViewerModel("pinocchio")
    robot.display(q0)
