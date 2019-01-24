from __future__ import print_function

import numpy as np
import pinocchio

model = pinocchio.buildSampleModelManipulator()
data  = model.createData()

JOINT_ID = 6
xdes     = np.matrix([ 0.5,-0.5,0.5]).T

q        = model.neutralConfiguration
eps      = 1e-4
IT_MAX   = 1000
DT       = 1e-1

for i in range(IT_MAX):
    pinocchio.forwardKinematics(model,data,q)
    x   = data.oMi[JOINT_ID].translation
    R   = data.oMi[JOINT_ID].rotation
    err = R.T*(x-xdes)
    if np.linalg.norm(err) < eps:
        print("Convergence achieved!")
        break
    J   = pinocchio.jointJacobian(model,data,q,JOINT_ID,pinocchio.ReferenceFrame.LOCAL,True)[:3,:]
    v   = - np.linalg.pinv(J)*err
    q   = pinocchio.integrate(model,q,v*DT)
    if not i % 10:        print('error = %s' % (x-xdes).T)
else:
    print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")

print('\nresult: %s' % q.flatten().tolist())
print('\nfinal error: %s' % (x-xdes).T)
