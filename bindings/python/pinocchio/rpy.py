#
# Copyright (c) 2015 CNRS
#

from math import atan2, pi, sqrt

import numpy as np

from . import libpinocchio_pywrap as pin


def npToTTuple(M):
    L = M.tolist()
    for i in range(len(L)):
        L[i] = tuple(L[i])
    return tuple(L)


def npToTuple(M):
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
    u = np.matrix(np.zeros([3, 1]), np.double)
    u[cood[axis]] = 1.0
    return pin.AngleAxis(ang, u).matrix()


def rpyToMatrix(rpy):
    '''
    # Convert from Roll, Pitch, Yaw to transformation Matrix
    '''
    return rotate('z', rpy[2, 0]).dot(rotate('y', rpy[1, 0])).dot(rotate('x', rpy[0, 0]))


def matrixToRpy(M):
    '''
    # Convert from Transformation Matrix to Roll, Pitch, Yaw
    '''
    m = sqrt(M[2, 1] ** 2 + M[2, 2] ** 2)
    p = atan2(-M[2, 0], m)

    if abs(abs(p) - pi / 2) < 0.001:
        r = 0
        y = -atan2(M[0, 1], M[1, 1])
    else:
        y = atan2(M[1, 0], M[0, 0])  # alpha
        r = atan2(M[2, 1], M[2, 2])  # gamma

    lst = [[r], [p], [y]]
    is_matrix = isinstance(M, np.matrix)
    return np.matrix(lst) if is_matrix else np.array(lst)
