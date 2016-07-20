#
# Copyright (c) 2015-2016 CNRS
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

import libpinocchio_pywrap as se3
from robot_wrapper import RobotWrapper


class RomeoWrapper(RobotWrapper):

    def __init__(self, filename, package_dirs=None, root_joint=None):
        RobotWrapper.__init__(self, filename, package_dirs=package_dirs, root_joint=root_joint)
        self.q0 = np.matrix([
            0, 0, 0.840252, 0, 0, 0, 1,                      # Free flyer
            0, 0, -0.3490658, 0.6981317, -0.3490658, 0,      # left leg
            0, 0, -0.3490658, 0.6981317, -0.3490658, 0,      # right leg
            0,                                               # chest
            1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2,         # left arm
            0, 0, 0, 0,                                      # head
            1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2,          # right arm
        ]).T

        self.opCorrespondances = {"lh": "LWristPitch",
                                  "rh": "RWristPitch",
                                  "rf": "RAnkleRoll",
                                  "lf": "LAnkleRoll",
                                  }

        for op, name in self.opCorrespondances.items():
            self.__dict__[op] = self.index(name)
            # self.__dict__['_M'+op] = types.MethodType(lambda s, q: s.position(q,idx),self)

    # --- SHORTCUTS ---
    def Mrh(self, q):
        return self.position(q, self.rh)

    def Jrh(self, q):
        return self.jacobian(q, self.rh)

    def wJrh(self, q):
        return se3.jacobian(self.model, self.data, self.rh, q, False)

    def vrh(self, q, v):
        return self.velocity(q, v, self.rh)

    def Jlh(self, q):
        return self.jacobian(q, self.lh)

    def Mlh(self, q):
        return self.position(q, self.lh)

    def Jlf(self, q):
        return self.jacobian(q, self.lf)

    def Mlf(self, q):
        return self.position(q, self.lf)

    def Jrf(self, q):
        return self.jacobian(q, self.rf)

    def Mrf(self, q):
        return self.position(q, self.rf)

__all__ = ['RomeoWrapper']
