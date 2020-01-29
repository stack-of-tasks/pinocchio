#
# Copyright (c) 2015 CNRS
#

import numpy as np

from . import libpinocchio_pywrap as pin


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


def rotate(axis, ang):
    '''
    # Transformation Matrix corresponding to a rotation about x,y or z
    eg. T = rot('x', pi / 4): rotate pi/4 rad about x axis
    '''
    cood = {'x': 0, 'y': 1, 'z': 2}
    u = np.zeros(3)
    u[cood[axis]] = 1.0
    return pin.AngleAxis(ang, u).matrix()


rpyToMatrix = pin._rpyToMatrix
matrixToRpy = pin._matrixToRpy

__all__ = ['npToTTuple', 'npToTuple', 'rotate', 'rpyToMatrix', 'matrixToRpy']
