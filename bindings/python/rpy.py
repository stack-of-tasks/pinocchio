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

from math import atan2, pi, sqrt

import numpy as np

import libpinocchio_pywrap as se3


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
    return se3.AngleAxis(ang, u).matrix()


def rpyToMatrix(rpy):
    '''
    # Convert from Roll, Pitch, Yaw to transformation Matrix
    '''
    return rotate('z', rpy[2, 0]) * rotate('y', rpy[1, 0]) * rotate('x', rpy[0, 0])


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

    return np.matrix([r, p, y], np.double).T
