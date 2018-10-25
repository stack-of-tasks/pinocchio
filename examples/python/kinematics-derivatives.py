##
## Copyright (c) 2018 CNRS
##
## This file is part of Pinocchio
## Pinocchio is free software: you can redistribute it
## and/or modify it under the terms of the GNU Lesser General Public
## License as published by the Free Software Foundation, either version
## 3 of the License, or (at your option) any later version.
##
## Pinocchio is distributed in the hope that it will be
## useful, but WITHOUT ANY WARRANTY; without even the implied warranty
## of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
## General Lesser Public License for more details. You should have
## received a copy of the GNU Lesser General Public License along with
## Pinocchio If not, see
## <http:##www.gnu.org/licenses/>.
##

import pinocchio as pin
import numpy as np

# Create model and data

model = pin.buildSampleModelHumanoidRandom()
data = model.createData()

# Set bounds (by default they are undefinded)

model.lowerPositionLimit = -np.matrix(np.ones((model.nq,1)))
model.upperPositionLimit = np.matrix(np.ones((model.nq,1)))

q = pin.randomConfiguration(model) # joint configuration
v = np.matrix(np.random.rand(model.nv,1)) # joint velocity
a = np.matrix(np.random.rand(model.nv,1)) # joint acceleration

# Evaluate all the terms required by the kinematics derivatives

pin.computeForwardKinematicsDerivatives(model,data,q,v,a)

# Evaluate the derivatives for a precise joint (e.g. rleg6_joint)

joint_name = "rleg6_joint"
joint_id = model.getJointId(joint_name)

# Derivatives of the spatial velocity with respect to the joint configuration and velocity vectors

(dv_dq,dv_dv) = pin.getJointVelocityDerivatives(model,data,joint_id,pin.ReferenceFrame.WORLD)
# or to get them in the LOCAL frame of the joint
(dv_dq_local,dv_dv_local) = pin.getJointVelocityDerivatives(model,data,joint_id,pin.ReferenceFrame.LOCAL)

# Derivatives of the spatial acceleration of the joint with respect to the joint configuration, velocity and acceleration vectors

(dv_dq,da_dq,da_dv,da_da) = pin.getJointAccelerationDerivatives(model,data,joint_id,pin.ReferenceFrame.WORLD)
# or to get them in the LOCAL frame of the joint
(dv_dq_local,da_dq_local,da_dv_local,da_da_local) = pin.getJointAccelerationDerivatives(model,data,joint_id,pin.ReferenceFrame.LOCAL)
