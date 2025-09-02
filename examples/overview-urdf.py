import os
from pathlib import Path
from sys import argv

import pinocchio

# This path refers to Pinocchio source code but you can define your own directory here.
model_dir = Path(os.environ.get("EXAMPLE_ROBOT_DATA_MODEL_DIR"))

# You should change here to set up your own URDF file or just pass it as an argument of
# this example.
urdf_filename = (
    model_dir / "ur_description/urdf/ur5_robot.urdf" if len(argv) < 2 else argv[1]
)

# Load the urdf model
model = pinocchio.buildModelFromUrdf(urdf_filename)
print("model name: " + model.name)

# Create data required by the algorithms
data = model.createData()

# Sample a random configuration
q = pinocchio.randomConfiguration(model)
print(f"q: {q.T}")

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))
