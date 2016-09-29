#
# Copyright (c) 2015 CNRS
#
# This file is part of Pinocchio
# Pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# Pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# Pinocchio If not, see
# <http://www.gnu.org/licenses/>.

import sys

import numpy as np
import numpy.linalg as npl

import libpinocchio_pywrap as se3
from rpy import matrixToRpy, npToTTuple, npToTuple, rotate, rpyToMatrix

eye = lambda n: np.matrix(np.eye(n), np.double)
zero = lambda n: np.matrix(np.zeros([n, 1] if isinstance(n, int) else n), np.double)
rand = lambda n: np.matrix(np.random.rand(n, 1) if isinstance(n, int) else np.random.rand(n[0], n[1]), np.double)


def cross(a, b):
    return np.matrix(np.cross(a.T, b.T).T, np.double)


def skew(p):
    x, y, z = p
    return np.matrix([[0, -z, y], [z, 0, -x], [-y, x, 0]], np.double)


def se3ToXYZQUAT(M):
    '''
    Convert the input SE3 object to a 7D tuple of floats [X,Y,Z,Q1,Q2,Q3,Q4] .
    '''
    xyz = M.translation
    quat = se3.Quaternion(M.rotation).coeffs()
    return [float(xyz[0, 0]), float(xyz[1, 0]), float(xyz[2, 0]),
            float(quat[0, 0]), float(quat[1, 0]), float(quat[2, 0]), float(quat[3, 0])]


def XYZQUATToSe3(xyzq):
    '''
    Reverse function of se3ToXYZQUAT: convert [X,Y,Z,Q1,Q2,Q3,Q4] to a SE3 element
    '''
    if isinstance(xyzq, (tuple, list)):
        xyzq = np.matrix(xyzq, np.float).T
    return se3.SE3(se3.Quaternion(xyzq[6, 0], xyzq[3, 0], xyzq[4, 0], xyzq[5, 0]).matrix(), xyzq[:3])


def XYZQUATToViewerConfiguration(xyzq):
    '''
    Convert the input 7D vector [X,Y,Z,x,y,z,w] to 7D vector [X,Y,Z,w,x,y,z]
    '''
    if isinstance(xyzq, (tuple, list)):
        xyzq = np.matrix(xyzq, np.float).T
    return [float(xyzq[0, 0]), float(xyzq[1, 0]), float(xyzq[2, 0]),
            float(xyzq[6, 0]), float(xyzq[3, 0]), float(xyzq[4, 0]), float(xyzq[5, 0])]


def ViewerConfigurationToXYZQUAT(vconf):
    '''
    Reverse function of XYZQUATToViewerConfiguration : convert [X,Y,Z,w,x,y,z] to [X,Y,Z,x,y,z,w]
    '''
    if isinstance(vconf, (tuple, list)):
        vconf = np.matrix(vconf, np.float).T
    return [float(vconf[0, 0]), float(vconf[1, 0]), float(vconf[2, 0]),
            float(vconf[4, 0]), float(vconf[5, 0]), float(vconf[6, 0]), float(vconf[3, 0])]


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
    if isinstance(M, se3.SE3):
        M = M.homogeneous
    ncol = M.shape[1]
    NC = 6
    print name, " = "
    print

    Mmin = lambda M: M.min() if np.nonzero(M)[1].shape[1]>0 else M.sum()
    Mmax = lambda M: M.max() if np.nonzero(M)[1].shape[1]>0 else M.sum()
    Mm = Mmin(abs(M[np.nonzero(M)]))
    MM = Mmax(abs(M[np.nonzero(M)]))

    fmt = "% 10.3e" if Mm < 1e-5 or MM > 1e6 or MM / Mm > 1e3 else "% 1.5f"

    for i in range((ncol - 1) / NC + 1):
        cmin = i * 6
        cmax = (i + 1) * 6
        cmax = ncol if ncol < cmax else cmax
        print "Columns %s through %s" % (cmin, cmax - 1)
        print
        for r in range(M.shape[0]):
            sys.stdout.write("  ")
            for c in range(cmin, cmax):
                if abs(M[r,c])>eps: sys.stdout.write(fmt % M[r,c]  + "   ")
                else: sys.stdout.write(" 0"+" "*9)
            print
        print


def fromListToVectorOfString(items):
    vector = se3.StdVec_StdString()
    vector.extend(item for item in items)
    return vector


__all__ = ['np', 'npl', 'eye', 'zero', 'rand', 'isapprox', 'mprint',
           'skew', 'cross',
           'npToTTuple', 'npToTuple', 'rotate',
           'rpyToMatrix', 'matrixToRpy',
           'se3ToXYZQUAT', 'XYZQUATToSe3',
           'XYZQUATToViewerConfiguration', 'ViewerConfigurationToXYZQUAT', 'fromListToVectorOfString']
