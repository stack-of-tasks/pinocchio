import numpy
import pinocchio

filename = "ur5.urdf" 
model = pinocchio.buildModelFromUrdf(filename)
data  = model.createData()

JOINT_ID = 6
DT       = 1e-1
q        = numpy.matrix([ 2, -.5, 1.8, 1.8, 2.6, -2 ]).T
xdes     = numpy.matrix([ 0,-0.5,0.5]).T

for i in range(100):
    J   = pinocchio.jointJacobian(model,data,q,JOINT_ID,pinocchio.ReferenceFrame.LOCAL,True)[:3,:]
    x   = data.oMi[JOINT_ID].translation
    R   = data.oMi[JOINT_ID].rotation
    err = R.T*(x-xdes)
    v   = - numpy.linalg.pinv(J)*err
    q   = pinocchio.integrate(model,q,v*DT)
    if not i % 10:        print 'error = ', (x-xdes).T
