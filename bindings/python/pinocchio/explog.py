#
# Copyright (c) 2015-2018 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

import math

import numpy as np

from . import pinocchio_pywrap as pin

def exp(x):
    if isinstance(x, pin.Motion):
        return pin.exp6(x)
    if np.isscalar(x):
        return math.exp(x)
    if isinstance(x, np.ndarray):
        if x.shape == (6, 1) or x.shape == (6,):
            return pin.exp6(pin.Motion(x))
        if x.shape == (3, 1) or x.shape == (3,):
            return pin.exp3(x)
        raise ValueError('Error only 3 and 6 vectors are allowed.')
    raise ValueError('Error exp is only defined for real, vector3, vector6 and pin.Motion objects.')


def log(x):
    if isinstance(x, pin.SE3):
        return pin.log6(x)
    if np.isscalar(x):
        return math.log(x)
    if isinstance(x, np.ndarray):
        if x.shape == (4, 4):
            return pin.log6(x)
        if x.shape == (3, 3):
            return pin.log3(x)
        raise ValueError('Error only 3 and 4 matrices are allowed.')
    raise ValueError('Error log is only defined for real, matrix3, matrix4 and pin.SE3 objects.')

__all__ = ['exp', 'log']
