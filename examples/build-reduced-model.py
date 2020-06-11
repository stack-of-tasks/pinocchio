import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import example_robot_data
import numpy as np
import os

# Goal: Build a reduced model from an existing URDF model by fixing the desired joints at a specified position.

# Load UR robot arm
modelPath = os.path.join(os.environ.get('HOME'), "Dev")
URDF_FILENAME = "ur5_robot.urdf"
URDF_SUBPATH = "/ur_description/urdf/" + URDF_FILENAME
robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, [modelPath])

# Check dimensions of the original model
print('standard model: dim=' + str(len(robot.model.joints)))
for jn in robot.model.joints:
    print(jn)

# Create a list of joints to lock
jointsToLock = ['wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Get the ID of all existing joints
jointsToLockIDs = []
for jn in jointsToLock:
    if robot.model.existJointName(jn):
        jointsToLockIDs.append(robot.model.getJointId(jn))
    else:
        print('Warning: joint ' + str(jn) + ' does not belong to the model!')

# Set initial position of both fixed and revoulte joints 
initialJointConfig = np.matrix([0,0,0,    # shoulder and elbow      
                                1,1,1]).T # gripper)

# Option 1: Build the reduced model including the geometric model for proper displaying of the robot
robot.model, robot.visual_model = pin.buildReducedModel(robot.model, robot.visual_model, jointsToLockIDs, initialJointConfig)

# Option 2: Only build the reduced model in case no display needed:
# robot.model = pin.buildReducedModel(robot.model, jointsToLockIDs, initialJointConfig)

# Check dimensions of the reduced model
print('reduced model: dim=' + str(len(robot.model.joints)))
for jn in robot.model.joints:
    print(jn)