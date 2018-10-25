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

##
## In this short script, we show how to compute the derivatives of the
## forward dynamics, using the algorithms explained in:
##
## Analytical Derivatives of Rigid Body Dynamics Algorithms, Justin Carpentier and Nicolas Mansard, Robotics: Science and Systems, 2018
##

# Create model and data

model = pin.buildSampleModelHumanoidRandom()
data = model.createData()

# Set bounds (by default they are undefinded for a the Simple Humanoid model)

model.lowerPositionLimit = -np.matrix(np.ones((model.nq,1)))
model.upperPositionLimit = np.matrix(np.ones((model.nq,1)))

q = pin.randomConfiguration(model) # joint configuration
v = np.matrix(np.random.rand(model.nv,1)) # joint velocity
tau = np.matrix(np.random.rand(model.nv,1)) # joint acceleration

# Evaluate the derivatives

pin.computeABADerivatives(model,data,q,v,tau)

# Retrieve the derivatives in data

ddq_dq = data.ddq_dq # Derivatives of the FD w.r.t. the joint config vector
ddq_dv = data.ddq_dv # Derivatives of the FD w.r.t. the joint velocity vector
ddq_dtau = data.Minv # Derivatives of the FD w.r.t. the joint acceleration vector
