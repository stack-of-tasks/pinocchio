#
# Copyright (c) 2015 CNRS
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

import math

import numpy as np

from . import libpinocchio_pywrap as se3


def exp(x):
    if isinstance(x, se3.Motion):
        return se3.exp6FromMotion(x)
    if np.isscalar(x):
        return math.exp(x)
    if isinstance(x, np.ndarray):
        if x.shape == (6, 1):
            return se3.exp6FromVector(x)
        if x.shape == (3, 1):
            return se3.exp3(x)
        raise ValueError('Error only 3 and 6 vectors are allowed.')
    raise ValueError('Error exp is only defined for real, vector3, vector6 and se3.Motion objects.')


def log(x):
    if isinstance(x, se3.SE3):
        return se3.log6FromSE3(x)
    if np.isscalar(x):
        return math.log(x)
    if isinstance(x, np.ndarray):
        if x.shape == (4, 4):
            return se3.log6FromMatrix(x)
        if x.shape == (3, 3):
            return se3.log3(x)
        raise ValueError('Error only 3 and 4 matrices are allowed.')
    raise ValueError('Error log is only defined for real, matrix3, matrix4 and se3.SE3 objects.')

__all__ = ['exp', 'log']
