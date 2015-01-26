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
            'npToTTuple','npToTuple','rotate','rpyToMatrix','matrixToRpy' ]
