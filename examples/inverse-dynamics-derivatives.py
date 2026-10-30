import numpy as np
import pinocchio as pin

##
## In this short script, we show how to compute the derivatives of the
## inverse dynamics (RNEA), using the algorithms proposed in:
##
## Analytical Derivatives of Rigid Body Dynamics Algorithms,
## Justin Carpentier and Nicolas Mansard,
## Robotics: Science and Systems, 2018
##

# Create model and data

model = pin.buildSampleModelHumanoidRandom()
data = model.createData()

# Set bounds (by default they are undefinded for a the Simple Humanoid model)

model.lowerPositionLimit = -np.ones(model.nq)
model.upperPositionLimit = np.ones(model.nq)

q = pin.randomConfiguration(model)  # joint configuration
v = np.random.rand(model.nv)  # joint velocity
a = np.random.rand(model.nv)  # joint acceleration

# Evaluate the derivatives

pin.computeRNEADerivatives(model, data, q, v, a)

# Retrieve the derivatives in data

dtau_dq = data.dtau_dq  # Derivatives of the ID w.r.t. the joint config vector
dtau_dv = data.dtau_dv  # Derivatives of the ID w.r.t. the joint velocity vector
dtau_da = data.M  # Derivatives of the ID w.r.t. the joint acceleration vector
