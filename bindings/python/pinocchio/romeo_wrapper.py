#
# Copyright (c) 2015-2016 CNRS
#

import numpy as np

from . import pinocchio_pywrap as pin
from .robot_wrapper import RobotWrapper


class RomeoWrapper(RobotWrapper):

    def __init__(self, filename, package_dirs=None, verbose=False):
        self.initFromURDF(filename, package_dirs=package_dirs, root_joint=pin.JointModelFreeFlyer(), verbose=verbose)
        self.q0 = np.array([
            0, 0, 0.840252, 0, 0, 0, 1,                      # Free flyer
            0, 0, -0.3490658, 0.6981317, -0.3490658, 0,      # left leg
            0, 0, -0.3490658, 0.6981317, -0.3490658, 0,      # right leg
            0,                                               # chest
            1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2,         # left arm
            0, 0, 0, 0,                                      # head
            1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2,          # right arm
        ])
        if pin.getNumpyType()==np.matrix:
            self.q0 = np.matrix(self.q0).T

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
        return pin.jacobian(self.model, self.data, self.rh, q, False)

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
