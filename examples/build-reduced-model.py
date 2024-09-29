from pathlib import Path

import numpy as np
import pinocchio as pin

# Goal: Build a reduced model from an existing URDF model by fixing the desired joints
# at a specified position.

# Load UR robot arm
# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = Path(__file__).parent.parent / "models"
model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir
# You should change here to set up your own URDF file
urdf_filename = model_path / "ur_description/urdf/ur5_robot.urdf"
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_filename, mesh_dir)

# Check dimensions of the original model
print("standard model: dim=" + str(len(model.joints)))
for jn in model.joints:
    print(jn)
print("-" * 30)

# Create a list of joints to lock
jointsToLock = ["wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# Get the ID of all existing joints
jointsToLockIDs = []
for jn in jointsToLock:
    if model.existJointName(jn):
        jointsToLockIDs.append(model.getJointId(jn))
    else:
        print("Warning: joint " + str(jn) + " does not belong to the model!")

# Set initial position of both fixed and revoulte joints
initialJointConfig = np.array(
    [
        0,
        0,
        0,  # shoulder and elbow
        1,
        1,
        1,
    ]
)  # gripper)

# Option 1: Only build the reduced model in case no display needed:
model_reduced = pin.buildReducedModel(model, jointsToLockIDs, initialJointConfig)

# Option 2: Build the reduced model including the geometric model for proper displaying
# of the robot.
model_reduced, visual_model_reduced = pin.buildReducedModel(
    model, visual_model, jointsToLockIDs, initialJointConfig
)

# Option 3: Build the reduced model including multiple geometric models (for example:
# visuals, collision).
geom_models = [visual_model, collision_model]
model_reduced, geometric_models_reduced = pin.buildReducedModel(
    model,
    list_of_geom_models=geom_models,
    list_of_joints_to_lock=jointsToLockIDs,
    reference_configuration=initialJointConfig,
)
# geometric_models_reduced is a list, ordered as the passed variable "geom_models" so:
visual_model_reduced, collision_model_reduced = (
    geometric_models_reduced[0],
    geometric_models_reduced[1],
)

# Check dimensions of the reduced model
# options 1-3 only take joint ids
print("joints to lock (only ids):", jointsToLockIDs)
print("reduced model: dim=" + str(len(model_reduced.joints)))
print("-" * 30)

# Option 4: Build a reduced model of a robot using RobotWrapper
# reference_configuration is optional: if not provided, neutral configuration used
# you can even mix joint names and joint ids
mixed_jointsToLockIDs = [jointsToLockIDs[0], "wrist_2_joint", "wrist_3_joint"]
robot = pin.RobotWrapper.BuildFromURDF(urdf_filename, mesh_dir, mimic=True)
reduced_robot = robot.buildReducedRobot(
    list_of_joints_to_lock=mixed_jointsToLockIDs,
    reference_configuration=initialJointConfig,
)

# Check dimensions of the reduced model and joint info
print("mixed joints to lock (names and ids):", mixed_jointsToLockIDs)
print("RobotWrapper reduced model: dim=" + str(len(reduced_robot.model.joints)))
for jn in robot.model.joints:
    print(jn)
