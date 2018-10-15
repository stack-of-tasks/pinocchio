import numpy
import pinocchio

filename = "ur5.urdf" 
model    = pinocchio.buildModelFromUrdf(filename)
data     = model.createData()
q        = numpy.matrix([ 2, -.5, 1.8, 1.8, 2.6, -2 ]).T

pinocchio.forwardKinematics(model,data,q)

for k in range(model.njoints):
    print("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( model.names[k], *data.oMi[k].translation.T.flat ))
