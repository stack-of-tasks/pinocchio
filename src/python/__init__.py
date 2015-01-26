import numpy as np
import libpinocchio_pywrap as se3
import utils

se3.SE3.__repr__ = se3.SE3.__str__
se3.Motion.__repr__ = se3.Motion.__str__
se3.AngleAxis.__repr__ = lambda s: 'AngleAxis('+s.vector().__str__()+')'

# --- SE3 action ---
def SE3act(m,x):
    assert(isinstance(m,se3.SE3))
    if isinstance(x,np.ndarray):
        if x.shape[0]==3:
            return m.rotation*x+m.translation
        elif x.shape[0] == 4:
            return m.homogeneous()*x
        elif x.shape[0] == 6:
            return m.action()*x
        else: raise Exception('Error: m can only act on linear object of size 3, 4 and 6.')
    elif 'se3Action' in x.__class__.__dict__:
        return x.se3Action(m)
    else:
        #print 'Error: SE3 cannot act on the given object'
        return m.oldmult(x)
setattr(se3.SE3,'oldmult',se3.SE3.__mul__)
setattr(se3.SE3,'__mul__',SE3act)
setattr(se3.SE3,'act',SE3act)

def SE3actinv(m,x):
    assert(isinstance(m,se3.SE3))
    if isinstance(x,np.ndarray):
        if x.shape[0]==3:
            return m.rotation.T*x-m.rotation.T*m.translation
        elif x.shape[0] == 4:
            return m.inverse().homogeneous()*x
        elif x.shape[0] == 6:
            return m.inverse().action()*x
        else: raise Exception('Error: m can only act on linear object of size 3, 4 and 6.')
    elif 'se3Action' in x.__class__.__dict__:
        return x.se3ActionInverse(m)
    else:
        print 'Error: SE3 cannot act on the given object'
setattr(se3.SE3,'actInv',SE3actinv)

# --- M6/F6 cross product --
def SE3cross(self,y):
    assert(isinstance(self,se3.Motion))
    if isinstance(y,se3.Motion):
        return self.cross_motion(y)
    elif isinstance(y,se3.Force):
        return self.cross_force(y)
    else: raise Exception('Error: SE3 cross product only apply on M6xM6 or M6xF6.')
setattr(se3.Motion,'__pow__',SE3cross)
setattr(se3.Motion,'cross',SE3cross)

from libpinocchio_pywrap import *
from pinocchio.robot_wrapper import RobotWrapper
from explog import exp,log

