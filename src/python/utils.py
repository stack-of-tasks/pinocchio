import numpy as np
import numpy.linalg as npl

eye  = lambda n: np.matrix(np.eye(n),np.double)
zero = lambda n: np.matrix(np.zeros([n,1] if isinstance(n,int) else n),np.double)
rand = lambda n: np.matrix(np.random.rand(n,1) if isinstance(n,int) else np.random.rand(n[0],n[1]),np.double)

def skew(p):
    x=p[0];y=p[1];z=p[2]
    return np.matrix([ [ 0,-z,y ], [ z,0,-x ], [ -y,x,0 ] ],np.double)

def isapprox(a,b,epsilon=1e-6):
    if issubclass(a.__class__,np.ndarray) and issubclass(b.__class__,np.ndarray):
        return np.allclose(a,b,epsilon)
    else:
        return abs(a-b)<epsilon
