#
# Copyright (c) 2016 CNRS
#

import numpy as np
from pinocchio import Motion, Force, skew
from pinocchio.utils import zero

def jFromIdx(idxv,robot):
    '''Return the joint index from the velocity index'''
    for j in range(1,robot.model.njoint):
        if idxv in range(robot.model.joints[j].idx_v,
                         robot.model.joints[j].idx_v+robot.model.joints[j].nv):
            return j

parent = lambda i,robot: robot.model.parents[i]
iv = lambda i,robot: list(range(robot.model.joints[i].idx_v,
                     robot.model.joints[i].idx_v+robot.model.joints[i].nv))
ancestors = lambda j,robot,res=[]: res if j==0 else ancestors(robot.model.parents[j],robot,[j,]+res)

class ancestorOf:
    def __init__(self,i,robot): self.dec=i; self.robot=robot
    def __contains__(self,anc):
        dec = self.dec
        while(dec>0):
            if anc==dec: return True
            else: dec = self.robot.model.parents[dec]
#descendants = lambda root,robot: filter( lambda i: root in ancestorOf(i,robot), range(root,robot.model.njoints) )
descendants = lambda root,robot: robot.model.subtrees[root]


def setRobotArgs(robot):
    ancestors.__defaults__ = (robot,)+ancestors.__defaults__ 
    descendants.__defaults__ = (robot,)
    #ancestorsOf.__init__.__defaults__ = (robot,)
    iv.__defaults__ = (robot,)
    parent.__defaults__ = (robot,)
    jFromIdx.__defaults__ = (robot,)

# --- SE3 operators
Mcross = lambda x,y: Motion(x).cross(Motion(y)).vector
Mcross.__doc__ = "Motion cross product"

Fcross = lambda x,y: Motion(x).cross(Force(y)).vector
Fcross.__doc__ = "Force cross product"

MCross = lambda V,v: np.bmat([ Mcross(V[:,i],v) for i in range(V.shape[1]) ])
FCross = lambda V,f: np.bmat([ Fcross(V[:,i],f) for i in range(V.shape[1]) ])


adj = lambda nu: np.bmat([[ skew(nu[3:]),skew(nu[:3])],[zero([3,3]),skew(nu[3:])]])
adj.__doc__ = "Motion pre-cross product (ie adjoint, lie bracket operator)"

adjdual = lambda nu: np.bmat([[ skew(nu[3:]),zero([3,3])],[skew(nu[:3]),skew(nu[3:])]])
adjdual.__doc__ = "Force pre-cross product adjdual(a) = -adj(a)' "

td = np.tensordot
quad = lambda H,v: np.matrix(td(td(H,v,[2,0]),v,[1,0])).T
quad.__doc__ = '''Tensor product v'*H*v, with H.shape = [ nop, nv, nv ]'''

def np_prettyprint(sarg = '{: 0.5f}',eps=5e-7):
    mformat = lambda  x,sarg = sarg,eps=eps: sarg.format(x) if abs(x)>eps else ' 0.     '
    np.set_printoptions(formatter={'float': mformat})

