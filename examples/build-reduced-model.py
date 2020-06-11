import pinocchio as pin
import numpy as np
from os.path import*

# Goal: Build a reduced model from an existing URDF model by fixing the desired joints at a specified position.

# Load UR robot arm
# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = join(dirname(os.dirname(str(abspath(__file__)))), "models")
model_path = pinocchio_model_dir + '/others/robots'
mesh_dir = model_path
# You should change here to set up your own URDF file
urdf_filename = model_path + '/ur_description/urdf/ur5_robot.urdf'
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_filename, mesh_dir)

# Check dimensions of the original model
print('standard model: dim=' + str(len(model.joints)))
for jn in model.joints:
    print(jn)

# Create a list of joints to lock
jointsToLock = ['wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Get the ID of all existing joints
jointsToLockIDs = []
for jn in jointsToLock:
    if model.existJointName(jn):
        jointsToLockIDs.append(model.getJointId(jn))
    else:
        print('Warning: joint ' + str(jn) + ' does not belong to the model!')

# Set initial position of both fixed and revoulte joints
initialJointConfig = np.array([0,0,0,   # shoulder and elbow
                                1,1,1]) # gripper)

# Option 1: Build the reduced model including the geometric model for proper displaying of the robot
model_reduced, visual_model_reduced = pin.buildReducedModel(model, visual_model, jointsToLockIDs, initialJointConfig)

# Option 2: Only build the reduced model in case no display needed:
# model_reduced = pin.buildReducedModel(model, jointsToLockIDs, initialJointConfig)

# Check dimensions of the reduced model
print('reduced model: dim=' + str(len(model_reduced.joints)))
for jn in model_reduced.joints:
    print(jn)
