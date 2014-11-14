'''
Execute several trajectory interpolation for various se(3) velocities. The
purpose of this script is to exhibit the use of the exp and log map for
interpolating SE(3) movements.
'''

import robotviewer
viewer=robotviewer.client('XML-RPC')
try:
    viewer.updateElementConfig('RomeoTrunkYaw',[1,0,0,0,0,0])
except: 
    viewer.updateElementConfig = lambda a,b: a

def se3ToRpy(m):
    return M.translation.T.tolist()[0] + matrixToRpy(M.rotation).T.tolist()[0]

import pinocchio as se3
from pinocchio.utils import *
norm = npl.norm
import time

N = 1000
M = se3.SE3.Identity()

# Integrate a constant body velocity.
v = zero(3); v[2] = 1.0 / N
w = zero(3); w[1] = 1.0 / N
nu = se3.Motion( v, w)

for i in range(N):
    M = se3.exp(nu)*M
    viewer.updateElementConfig('RomeoTrunkYaw', se3ToRpy(M))
time.sleep(1)

# Integrate a velocity of the body that is constant in the world frame.
for i in range(N):
    Mc = se3.SE3(M.rotation,zero(3))
    M  = M*se3.exp(Mc.actInv(nu))
    viewer.updateElementConfig('RomeoTrunkYaw', se3ToRpy(M))
time.sleep(1)

# Integrate a constant "log" velocity in body frame.
ME = se3.SE3( se3.Quaternion(0.7, -0.6,  0.1,  0.4).normalized().matrix(), 
              np.matrix([1,-1,2],np.double).T )
nu = se3.Motion(se3.log(M.inverse()*ME).vector()/N)
for i in range(N):
    M = M*se3.exp(nu)
    viewer.updateElementConfig('RomeoTrunkYaw', se3ToRpy(M))
print "Residuals = ", norm( se3.log(M.inverse()*ME).vector() )
time.sleep(1)

# Integrate a constant "log" velocity in reference frame.
ME = se3.SE3( se3.Quaternion(0.3, -0.2,  0.6,  0.5).normalized().matrix(), 
              np.matrix([-1,1,0.6],np.double).T )
nu = se3.Motion(se3.log(M.inverse()*ME).vector()/N)
for i in range(N):
    M = M*se3.exp(nu)
    viewer.updateElementConfig('RomeoTrunkYaw', se3ToRpy(M))
print "Residuals = ", norm( se3.log(M.inverse()*ME).vector() )
time.sleep(1)

# Integrate an exponential decay vector field toward ME.
ME = se3.SE3( se3.Quaternion(0.9, -0.1,  0.1,  0.1).normalized().matrix(), 
              np.matrix([1,0.2,1.9],np.double).T )
for i in range(N):
    nu = se3.log(M.inverse()*ME).vector() * 1e-2
    M = M*se3.exp(nu)
    viewer.updateElementConfig('RomeoTrunkYaw', se3ToRpy(M))
print "Residuals = ", norm( se3.log(M.inverse()*ME).vector() )
time.sleep(1)

# Integrate a straight-line vector field toward ME.
ME = se3.SE3( se3.Quaternion(0.1, -0.6,  0.6,  0.1).normalized().matrix(), 
              np.matrix([.1,-1.2,0.3],np.double).T )
for i in range(N):
    ERR = ME.inverse()*M
    v = ERR.rotation.T*ERR.translation * -9e-3
    w = ERR.rotation.T*se3.log(ERR.rotation) * -9e-3
    nu = se3.Motion(v,w)
    M = M*se3.exp(nu)
    viewer.updateElementConfig('RomeoTrunkYaw', se3ToRpy(M))
print "Residuals = ", norm( se3.log(M.inverse()*ME).vector() )
time.sleep(1)

