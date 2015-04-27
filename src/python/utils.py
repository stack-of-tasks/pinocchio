import numpy as np
import numpy.linalg as npl

eye  = lambda n: np.matrix(np.eye(n),np.double)
zero = lambda n: np.matrix(np.zeros([n,1] if isinstance(n,int) else n),np.double)
rand = lambda n: np.matrix(np.random.rand(n,1) if isinstance(n,int) else np.random.rand(n[0],n[1]),np.double)

def cross(a,b):
    return np.matrix( np.cross(a.T,b.T).T ,np.double)
    

def skew(p):
    x=p[0];y=p[1];z=p[2]
    return np.matrix([ [ 0,-z,y ], [ z,0,-x ], [ -y,x,0 ] ],np.double)

# Convert the input SE3 object to a 7D tuple of floats [X,Y,Z,Q1,Q2,Q3,Q4] .
def se3ToXYZQUAT(M):
    xyz = M.translation
    quat = se3.Quaternion(M.rotation).coeffs()
    config = [  float(xyz[0,0]), float(xyz[1,0]), float(xyz[2,0]),
                float(quat[0,0]), float(quat[1,0]), float(quat[2,0]), float(quat[3,0]) ]
    return config

# Reverse function of se3ToXYZQUAT: convert [X,Y,Z,Q1,Q2,Q3,Q4] to a homogeneous matrix.
def XYZQUATToSe3(xyzq):
    if isinstance(xyzq,tuple) or isinstance(xyzq,list):
        xyzq = np.matrix(xyzq,np.float).T
    return se3.SE3(
        se3.Quaternion( xyzq[6,0],xyzq[3,0],xyzq[4,0],xyzq[5,0]).matrix(),
        xyzq[:3] )

# Convert the input 7D vector [X,Y,Z,b,c,d,a] to 7D vector [X,Y,Y,a,b,c,d]
def XYZQUATToViewerConfiguration(xyzq):
    if isinstance(xyzq,tuple) or isinstance(xyzq,list):
        xyzq = np.matrix(xyzq,np.float).T
    VConf = [   float(xyzq[0,0]), float(xyzq[1,0]), float(xyzq[2,0]),
                float(xyzq[6,0]), float(xyzq[3,0]), float(xyzq[4,0]), float(xyzq[5,0])  ]
    return VConf

# Reverse function of XYZQUATToViewerConfiguration : convert [X,Y,Z,a,b,c,d] to
def ViewerConfigurationToXYZQUAT(vconf):
    if isinstance(vconf,tuple) or isinstance(vconf,list):
        vconf = np.matrix(vconf,np.float).T
    xyzq = [    float(vconf[0,0]), float(vconf[1,0]), float(vconf[2,0]),
                float(vconf[6,0]), float(vconf[3,0]), float(vconf[4,0]), float(vconf[5,0])  ]
    return xyzq

def isapprox(a,b,epsilon=1e-6):
    if "np" in a.__class__.__dict__: a = a.np
    if "np" in b.__class__.__dict__: b = b.np
    if issubclass(a.__class__,np.ndarray) and issubclass(b.__class__,np.ndarray):
        return np.allclose(a,b,epsilon)
    else:
        return abs(a-b)<epsilon

import sys
def mprint(M,name = "ans"):
    '''
    Matlab-style pretty matrix print.
    '''
    if M.__class__ == se3.SE3: M = M.homogeneous
    ncol = M.shape[1]
    NC = 6
    print name," = "
    print ""
    
    Mm = abs(M[np.nonzero(M)]).min(); MM = abs(M[np.nonzero(M)]).max()
    if Mm<1e-2 or MM > 1e6 or MM/Mm > 1e3:
        fmt = "%.4e"
    else:
        f=np.log(MM*Mm)/np.log(10)
        #if f<0: fmt = "%1.5f"
        #elif f<2: fmt = 
        fmt = "% 1.5f"


    for i in range( (ncol-1) /  NC  +1 ):
        cmin = i*6
        cmax = (i+1)*6
        cmax = ncol if ncol<cmax else cmax
        print "Columns ",cmin," through ",cmax-1
        print ""
        for r in range(M.shape[0]):
            sys.stdout.write("  ")
            for c in range(cmin,cmax):
                #if M[r,c]>=0: sys.stdout.write('#')
                sys.stdout.write(fmt % M[r,c]+"   ")
            print ""
        print ""


from rpy import *

__all__ = [ 'np','npl','eye','zero','rand','isapprox','mprint',
            'skew', 'cross',
            'npToTTuple','npToTuple','rotate',
            'rpyToMatrix','matrixToRpy',
            'se3ToXYZQUAT' ,'XYZQUATToSe3',
            'XYZQUATToViewerConfiguration', 'ViewerConfigurationToXYZQUAT' ]
