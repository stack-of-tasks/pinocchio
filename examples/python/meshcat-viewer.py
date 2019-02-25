# pinocchio.RobotWrapper is interfaced with the web-based mesh viewer Meshcat.
# This examples shows how to load and move a robot in meshcat.
# Note: this feature requires Meshcat to be installed, this can be done using
# pip install --user meshcat

import pinocchio as se3
import numpy as np
import os
from future.builtins import input

from pinocchio.robot_wrapper import RobotWrapper

import meshcat

# Load the URDF model.
current_file =  os.path.dirname(os.path.abspath(__file__))
romeo_model_dir = os.path.abspath(os.path.join(current_file, '../../models/romeo'))
romeo_model_path = os.path.abspath(os.path.join(romeo_model_dir, 'romeo_description/urdf/romeo.urdf'))
robot = RobotWrapper.BuildFromURDF(romeo_model_path, [romeo_model_dir], se3.JointModelFreeFlyer())

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in a terminal:
# this enables the server to remain active after the current script ends.
vis = meshcat.Visualizer()

# Load the robot in the viewer.
# Color is needed here because the Romeo URDF doesn't contain any color, so the default color results in an
# invisible robot (alpha value set to 0).
robot.initMeshcatDisplay(vis, robot_color = [0.0, 0.0, 0.0, 1.0])

q = robot.q0
# Separate between freeflyer and robot links.
n_freeflyer = 7
n_links = len(q) - n_freeflyer
q[n_freeflyer:] = np.matrix(np.random.rand(n_links)).T
robot.display(q)
input("Displaying a single random robot configuration. Press enter to continue")

red_robot = RobotWrapper.BuildFromURDF(romeo_model_path, [romeo_model_dir], se3.JointModelFreeFlyer())
robot.initMeshcatDisplay(vis, robot_name = "red_robot", robot_color = [1.0, 0.0, 0.0, 0.5])
q[1] = 1.0
q[n_freeflyer:] = np.matrix(np.random.rand(n_links)).T
robot.display(q)
input("Displaying a second robot with color red, semi-transparent. Press enter to exit")
