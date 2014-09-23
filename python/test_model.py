import pinocchio as se3
from pinocchio.utils import *

model = se3.Model.BuildEmptyModel()
print model
model = se3.Model.BuildHumanoidSimple()
print model
print "Bye bye"

data = model.createData()

print model.inertias()
