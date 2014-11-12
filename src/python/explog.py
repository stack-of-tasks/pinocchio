import libpinocchio_pywrap as se3
import numpy as np
import numpy.linalg as npl
from math import sin,cos
import math
from utils import skew,cross,eye

def exp3(v):
    '''
    Exp: so3 -> SO3. Return the integral of the input angular velocity during time 1.
    '''
    nv = npl.norm(v)
    return se3.AngleAxis(nv,v/nv).matrix()

def log3(R):
    '''
    Log: SO3 -> so3. Pseudo-inverse of log from SO3 -> { v \in so3, ||v|| < 2pi }.
    '''
    return se3.AngleAxis(R).vector()

def exp6( nu ):
    '''
    Exp: se3 -> SE3. Return the integral of the input spatial velocity during time 1.
    '''
    if isinstance(nu,se3.Motion): w = nu.angular; v = nu.linear
    else:    w = nu[3:]; v = nu[:3]
    if npl.norm(w)>1e-15:
        R = exp3(w)
        t = npl.norm(w)
        S = skew(w)
        V = eye(3) + (1-cos(t))/t**2 * S + (t-sin(t))/t**3 * S*S
        p = V*v
        return se3.SE3(R,p)
    else:
        return se3.SE3(eye(3),v)

def log6( m ):
    '''
    Log: SE3 -> se3. Pseudo-inverse of exp from SE3 -> { v,w \in se3, ||w|| < 2pi }.
    '''
    if isinstance(m,se3.SE3): R = m.rotation; p = m.translation
    else: R = m[:3,:3]; p = m[:3,3]
    w = log3(R)
    if npl.norm(w)>1e-15:
        t = npl.norm(w)
        S = skew(w)
        V = eye(3) + (1-cos(t))/t**2 * S + (t-sin(t))/t**3 * S*S
        v = npl.inv(V)*p
    else:
        v = p
    return se3.Motion(v,w)

def exp(x):
    if isinstance(x,se3.Motion): return exp6(x)
    elif np.isscalar(x): return math.exp(x)
    elif isinstance(x,np.matrix):
        if len(x)==6: return exp6(x)
        elif len(x)==3: return exp3(x)
        else: print 'Error only 3 and 6 vectors are allowed.'
    else: print 'Error exp is only defined for real, vector3, vector6 and se3.Motion objects.'

def log(x):
    if isinstance(x,se3.SE3): return log6(x)
    elif np.isscalar(x): return math.log(x)
    elif isinstance(x,np.matrix):
        if len(x)==6: return log6(x)
        elif len(x)==3: return log3(x)
        else: print 'Error only 3 and 6 vectors are allowed.'
    else: print 'Error log is only defined for real, vector3, vector6 and se3.SE3 objects.'


__all__ = [ 'exp3', 'log3', 'exp6', 'log6','exp','log' ]
