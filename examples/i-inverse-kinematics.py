import numpy
import pinocchio
from sys import argv

filename = "ur5.urdf" if len(argv)<2 else argv[1]
model = pinocchio.buildModelFromUrdf(filename)
data  = model.createData()

JOINT_ID = 6
DT       = 1e-1
q        = model.neutralConfiguration
xdes     = numpy.matrix([ 0,-0.5,0.5]).T

for i in range(100):
    pinocchio.computeJointJacobians(model,data,q)
    J   = pinocchio.jointJacobian(model,data,q,JOINT_ID,pinocchio.ReferenceFrame.LOCAL,True)[:3,:]
    x   = data.oMi[JOINT_ID].translation
    R   = data.oMi[JOINT_ID].rotation
    err = R.T*(x-xdes)
    v   = - numpy.linalg.pinv(J)*err
    q   = pinocchio.integrate(model,q,v*DT)
    if not i % 10:        print 'error = ', (x-xdes).T

# Computing error for final q
pinocchio.forwardKinematics(model,data,q)
x   = data.oMi[JOINT_ID].translation
err = x-xdes
print '\nfinal error: ', err.T
