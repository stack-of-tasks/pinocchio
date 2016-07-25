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

import numpy as np
from pinocchio.robot_wrapper import RobotWrapper

import libpinocchio_pywrap as se3
import utils
from explog import exp, log
from libpinocchio_pywrap import *

se3.SE3.__repr__ = se3.SE3.__str__
se3.Motion.__repr__ = se3.Motion.__str__
se3.AngleAxis.__repr__ = lambda s: 'AngleAxis(%s)' % s.vector()


# --- SE3 action ---
def SE3act(m, x):
    assert isinstance(m, se3.SE3)
    if isinstance(x, np.ndarray):
        if x.shape[0] == 3:
            return m.rotation * x + m.translation
        if x.shape[0] == 4:
            return m.homogeneous * x
        if x.shape[0] == 6:
            return m.action * x
        raise ValueError('m can only act on linear object of size 3, 4 and 6.')
    if 'se3Action' in x.__class__.__dict__:
        return x.se3Action(m)
    return m.oldmult(x)

setattr(se3.SE3, 'oldmult', se3.SE3.__mul__)
setattr(se3.SE3, '__mul__', SE3act)
setattr(se3.SE3, 'act', SE3act)


def SE3actinv(m, x):
    assert isinstance(m, se3.SE3)
    if isinstance(x, np.ndarray):
        if x.shape[0] == 3:
            return m.rotation.T * x - m.rotation.T * m.translation
        if x.shape[0] == 4:
            return m.inverse().homogeneous * x
        if x.shape[0] == 6:
            return m.inverse().action * x
        raise ValueError('m can only act on linear object of size 3, 4 and 6.')
    if 'se3Action' in x.__class__.__dict__:
        return x.se3ActionInverse(m)
    raise ValueError('SE3 cannot act on the given object')

setattr(se3.SE3, 'actInv', SE3actinv)


# --- M6/F6 cross product --
def SE3cross(self, y):
    assert isinstance(self, se3.Motion)
    if isinstance(y, se3.Motion):
        return self.cross_motion(y)
    if isinstance(y, se3.Force):
        return self.cross_force(y)
    raise ValueError('SE3 cross product only apply on M6xM6 or M6xF6.')

setattr(se3.Motion, '__pow__', SE3cross)
setattr(se3.Motion, 'cross', SE3cross)
