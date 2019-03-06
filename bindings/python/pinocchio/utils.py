#
# Copyright (c) 2015-2019 CNRS INRIA
#

from __future__ import print_function

import sys

import numpy as np
import numpy.linalg as npl

from . import libpinocchio_pywrap as pin
from .rpy import matrixToRpy, npToTTuple, npToTuple, rotate, rpyToMatrix

from .deprecation import deprecated

eye = lambda n: np.matrix(np.eye(n), np.double)
zero = lambda n: np.matrix(np.zeros([n, 1] if isinstance(n, int) else n), np.double)
rand = lambda n: np.matrix(np.random.rand(n, 1) if isinstance(n, int) else np.random.rand(n[0], n[1]), np.double)


def cross(a, b):
    return np.matrix(np.cross(a.T, b.T).T, np.double)


def skew(p):
    x, y, z = p
    return np.matrix([[0, -z, y], [z, 0, -x], [-y, x, 0]], np.double)


@deprecated('Now useless. You can directly have access to this function from the main scope of Pinocchio')
def se3ToXYZQUAT(M):
    return pin.se3ToXYZQUATtuple(M)

@deprecated('Now useless. You can directly have access to this function from the main scope of Pinocchio')
def XYZQUATToSe3(vec):
    return pin.XYZQUATToSe3(vec)


@deprecated('Now useless.')
def XYZQUATToViewerConfiguration(xyzq):
    '''
    Convert the input 7D vector [X,Y,Z,x,y,z,w] to 7D vector [X,Y,Z,x,y,z,w]
    Gepetto Viewer Corba has changed its convention for quaternions - This function is not more required.
    See https://github.com/humanoid-path-planner/gepetto-viewer-corba/pull/58 for more details.
    '''
    if isinstance(xyzq, (np.matrix)):
        return xyzq.A.squeeze().tolist()
    return xyzq

@deprecated('Now useless.')
def ViewerConfigurationToXYZQUAT(vconf):
    '''
    Reverse function of XYZQUATToViewerConfiguration : convert [X,Y,Z,x,y,z,w] to [X,Y,Z,x,y,z,w]
    Gepetto Viewer Corba has changed its convention for quaternions - This function is not more required.
    See https://github.com/humanoid-path-planner/gepetto-viewer-corba/pull/58 for more details.
    '''
    return vconf


def isapprox(a, b, epsilon=1e-6):
    if "np" in a.__class__.__dict__:
        a = a.np
    if "np" in b.__class__.__dict__:
        b = b.np
    if isinstance(a, (np.ndarray, list)) and isinstance(b, (np.ndarray, list)):
        return np.allclose(a, b, epsilon)
    return abs(a - b) < epsilon


def mprint(M, name="ans",eps=1e-15):
    '''
    Matlab-style pretty matrix print.
    '''
    if isinstance(M, pin.SE3):
        M = M.homogeneous
    ncol = M.shape[1]
    NC = 6
    print(name, " = ")
    print()

    Mmin = lambda M: M.min()
    Mmax = lambda M: M.max()
    Mm = Mmin(abs(M[np.nonzero(M)]))
    MM = Mmax(abs(M[np.nonzero(M)]))

    fmt = "% 10.3e" if Mm < 1e-5 or MM > 1e6 or MM / Mm > 1e3 else "% 1.5f"

    for i in range((ncol - 1) / NC + 1):
        cmin = i * 6
        cmax = (i + 1) * 6
        cmax = ncol if ncol < cmax else cmax
        print("Columns %s through %s" % (cmin, cmax - 1))
        print()
        for r in range(M.shape[0]):
            sys.stdout.write("  ")
            for c in range(cmin, cmax):
                if abs(M[r,c])>eps: sys.stdout.write(fmt % M[r,c]  + "   ")
                else: sys.stdout.write(" 0"+" "*9)
            print()
        print()


def fromListToVectorOfString(items):
    vector = pin.StdVec_StdString()
    vector.extend(item for item in items)
    return vector


__all__ = ['np', 'npl', 'eye', 'zero', 'rand', 'isapprox', 'mprint',
           'skew', 'cross',
           'npToTTuple', 'npToTuple', 'rotate',
           'rpyToMatrix', 'matrixToRpy',
           'se3ToXYZQUAT', 'XYZQUATToSe3',
           'XYZQUATToViewerConfiguration', 'ViewerConfigurationToXYZQUAT', 'fromListToVectorOfString']
