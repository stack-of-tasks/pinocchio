from __future__ import print_function

import numpy as np
from numpy.linalg import norm, solve

import pinocchio

model = pinocchio.buildSampleModelManipulator()
data = model.createData()

JOINT_ID = 6
oMdes = pinocchio.SE3(np.eye(3), np.array([1.0, 0.0, 1.0]))

q = pinocchio.neutral(model)
eps = 1e-4
IT_MAX = 1000
DT = 1e-1
damp = 1e-12

it = 0
while True:
    pinocchio.forwardKinematics(model, data, q)
    iMd = data.oMi[JOINT_ID].actInv(oMdes)
    err = iMd.translation
    if norm(err) < eps:
        success = True
        break
    if it >= IT_MAX:
        success = False
        break
    J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame
    J = -J[:3, :]  # linear part of the Jacobian
    v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(3), err))
    q = pinocchio.integrate(model, q, v * DT)
    if not it % 10:
        print("%d: error = %s" % (it, err.T))
    it += 1

if success:
    print("Convergence achieved!")
else:
    print(
        "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
    )

print("\nresult: %s" % q.flatten().tolist())
print("\nfinal error: %s" % err.T)
