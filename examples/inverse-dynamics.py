# Copyright 2023 Inria
# SPDX-License-Identifier: BSD-2-Clause


"""
In this short script, we show how to compute inverse dynamics (RNEA), i.e. the
vector of joint torques corresponding to a given motion.
"""

from pathlib import Path

import numpy as np
import pinocchio as pin

# Load the model from a URDF file
# Change to your own URDF file here, or give a path as command-line argument
pinocchio_model_dir = Path(__file__).parent.parent / "models/"
model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
urdf_filename = "ur5_robot.urdf"
urdf_model_path = model_path / "ur_description/urdf/" / urdf_filename
model, _, _ = pin.buildModelsFromUrdf(urdf_model_path, package_dirs=mesh_dir)

# Build a data frame associated with the model
data = model.createData()

# Sample a random joint configuration, joint velocities and accelerations
q = pin.randomConfiguration(model)  # in rad for the UR5
v = np.random.rand(model.nv, 1)  # in rad/s for the UR5
a = np.random.rand(model.nv, 1)  # in rad/s² for the UR5

# Computes the inverse dynamics (RNEA) for all the joints of the robot
tau = pin.rnea(model, data, q, v, a)

# Print out to the vector of joint torques (in N.m)
print("Joint torques: " + str(tau))
