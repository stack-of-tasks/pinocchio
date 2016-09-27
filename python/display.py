import unittest
import pinocchio as se3
import numpy as np
import os

from pinocchio.robot_wrapper import RobotWrapper

# Warning : the paths are here hard-coded. This file is only here as an example
romeo_model_path = os.path.abspath(os.path.join(current_file, '../models/romeo'))
romeo_model_file = romeo_model_path + "/urdf/romeo.urdf"

list_hints = [romeo_model_path,"titi"]
robot = RobotWrapper(romeo_model_file,list_hints, se3.JointModelFreeFlyer())

robot.initDisplay()
robot.loadDisplayModel("world/pinocchio")

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

