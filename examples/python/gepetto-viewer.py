# NOTE: this example needs gepetto-gui to be installed
# usage: launch gepetto-gui and then run this test

import unittest
import pinocchio as pin
import numpy as np
import os

from pinocchio.robot_wrapper import RobotWrapper

# Load the URDF model.
current_file =  os.path.dirname(os.path.abspath(__file__))
romeo_model_dir = os.path.abspath(os.path.join(current_file, '../../models/romeo'))
romeo_model_path = os.path.abspath(os.path.join(romeo_model_dir, 'romeo_description/urdf/romeo_small.urdf'))
robot = RobotWrapper.BuildFromURDF(str(romeo_model_path), [str(romeo_model_dir)], pin.JointModelFreeFlyer())

# Initialize the robot display.
robot.initDisplay()
robot.loadDisplayModel("pinocchio")

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
robot.display(q0)
