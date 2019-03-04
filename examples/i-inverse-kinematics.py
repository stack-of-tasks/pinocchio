from __future__ import print_function

import numpy as np
import pinocchio

model = pinocchio.buildSampleModelManipulator()
data  = model.createData()

JOINT_ID = 6
xdes     = np.matrix([ 0.5,-0.5,0.5]).T

q        = pinocchio.neutral(model)
eps      = 1e-4
IT_MAX   = 1000
DT       = 1e-1

i=0
while True:
    pinocchio.forwardKinematics(model,data,q)
    x   = data.oMi[JOINT_ID].translation
    R   = data.oMi[JOINT_ID].rotation
    err = R.T*(x-xdes)
    if np.linalg.norm(err) < eps:
        print("Convergence achieved!")
        break
    if i >= IT_MAX:
        print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
        break
    J   = pinocchio.jointJacobian(model,data,q,JOINT_ID)[:3,:]
    v   = - np.linalg.pinv(J)*err
    q   = pinocchio.integrate(model,q,v*DT)
    if not i % 10:        print('error = %s' % err.T)
    i += 1

print('\nresult: %s' % q.flatten().tolist())
print('\nfinal error: %s' % err.T)
