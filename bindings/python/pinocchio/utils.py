#
# Copyright (c) 2015-2022 CNRS INRIA
#


import sys

import numpy as np
import numpy.linalg as npl

from . import pinocchio_pywrap_default as pin
from .pinocchio_pywrap_default.rpy import matrixToRpy, rotate, rpyToMatrix


def npToTTuple(M):
    L = M.tolist()
    for i in range(len(L)):
        L[i] = tuple(L[i])
    return tuple(L)


def npToTuple(M):
    if len(M.shape) == 1:
        return tuple(M.tolist())
    if M.shape[0] == 1:
        return tuple(M.tolist()[0])
    if M.shape[1] == 1:
        return tuple(M.T.tolist()[0])
    return npToTTuple(M)


def eye(n):
    res = np.eye(n)
    return res


def zero(n):
    return np.zeros(n)


def rand(n):
    return np.random.rand(n) if isinstance(n, int) else np.random.rand(n[0], n[1])


def isapprox(a, b, epsilon=1e-6):
    if "np" in a.__class__.__dict__:
        a = a.np
    if "np" in b.__class__.__dict__:
        b = b.np
    if isinstance(a, (np.ndarray, list)) and isinstance(b, (np.ndarray, list)):
        a = np.squeeze(np.array(a))
        b = np.squeeze(np.array(b))
        return np.allclose(a, b, epsilon)
    return abs(a - b) < epsilon


def mprint(M, name="ans", eps=1e-15):
    """
    Matlab-style pretty matrix print.
    """
    if isinstance(M, pin.SE3):
        M = M.homogeneous
    if len(M.shape) == 1:
        M = np.expand_dims(M, axis=0)
    ncol = M.shape[1]
    NC = 6
    print(name, " = ")
    print()

    Mm = (abs(M[np.nonzero(M)])).min()
    MM = (abs(M[np.nonzero(M)])).max()

    fmt = "% 10.3e" if Mm < 1e-5 or MM > 1e6 or MM / Mm > 1e3 else "% 1.5f"

    for i in range((ncol - 1) // NC + 1):
        cmin = i * 6
        cmax = (i + 1) * 6
        cmax = ncol if ncol < cmax else cmax
        print(f"Columns {cmin} through {cmax - 1}")
        print()
        for r in range(M.shape[0]):
            sys.stdout.write("  ")
            for c in range(cmin, cmax):
                if abs(M[r, c]) > eps:
                    sys.stdout.write(fmt % M[r, c] + "   ")
                else:
                    sys.stdout.write(" 0" + " " * 9)
            print()
        print()


def fromListToVectorOfString(items):
    vector = pin.StdVec_StdString()
    vector.extend(item for item in items)
    return vector


__all__ = [
    "eye",
    "fromListToVectorOfString",
    "isapprox",
    "matrixToRpy",
    "mprint",
    "np",
    "npToTTuple",
    "npToTuple",
    "npl",
    "rand",
    "rotate",
    "rpyToMatrix",
    "zero",
]
