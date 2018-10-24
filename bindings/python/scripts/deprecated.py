#
# Copyright (c) 2018 CNRS
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

## In this file, are reported some depracated functions that are still maintain until the next important release 1.4.0 ##

from __future__ import print_function

from . import libpinocchio_pywrap as se3 
from .deprecation import deprecated

@deprecated("This function has been deprecated. Please use buildSampleModelHumanoid or buildSampleModelHumanoidRandom instead.")
def buildSampleModelHumanoidSimple(usingFF=True):
    return se3.buildSampleModelHumanoidRandom(usingFF)

@deprecated("Static method Model.BuildHumanoidSimple has been deprecated. Please use function buildSampleModelHumanoid or buildSampleModelHumanoidRandom instead.")
def _Model__BuildHumanoidSimple(usingFF=True):
    return se3.buildSampleModelHumanoidRandom(usingFF)

se3.Model.BuildHumanoidSimple = staticmethod(_Model__BuildHumanoidSimple)

@deprecated("Static method Model.BuildEmptyModel has been deprecated. Please use the empty Model constructor instead.")
def _Model__BuildEmptyModel():
    return se3.Model()

se3.Model.BuildEmptyModel = staticmethod(_Model__BuildEmptyModel)

@deprecated("This function has been renamed updateFramePlacements when taking two arguments, and framesForwardKinematics when taking three. Please change your code to the appropriate method.")
def framesKinematics(model,data,q=None):
  if q is None:
    se3.updateFramePlacements(model,data)
  else:
    se3.framesForwardKinematics(model,data,q)

@deprecated("This function has been renamed computeJointJacobians and will be removed in release 1.4.0 of Pinocchio. Please change for new computeJointJacobians.")
def computeJacobians(model,data,q=None):
  if q is None:
    return se3.computeJointJacobians(model,data)
  else:
    return se3.computeJointJacobians(model,data,q)

@deprecated("This function has been renamed jointJacobian and will be removed in release 1.4.0 of Pinocchio. Please change for new jointJacobian.")
def jacobian(model,data,q,jointId,local,update_kinematics):
  if local:
    return se3.jointJacobian(model,data,q,jointId,se3.ReferenceFrame.LOCAL,update_kinematics)
  else:
    return se3.jointJacobian(model,data,q,jointId,se3.ReferenceFrame.WORLD,update_kinematics)

@deprecated("This function has been renamed getJointJacobian and will be removed in release 1.4.0 of Pinocchio. Please change for new getJointJacobian.")
def getJacobian(model,data,jointId,local):
  if local:
    return se3.getJointJacobian(model,data,jointId,se3.ReferenceFrame.LOCAL)
  else:
    return se3.getJointJacobian(model,data,jointId,se3.ReferenceFrame.WORLD)

@deprecated("This function has been renamed computeJacobiansTimeVariation and will be removed in release 1.4.0 of Pinocchio. Please change for new computeJointJacobiansTimeVariation.")
def computeJacobiansTimeVariation(model,data,q,v):
  return se3.computeJointJacobiansTimeVariation(model,data,q,v)

@deprecated("This function has been renamed getJointJacobianTimeVariation and will be removed in release 1.4.0 of Pinocchio. Please change for new getJointJacobianTimeVariation.")
def getJacobianTimeVariation(model,data,jointId,local):
  if local:
    return se3.getJointJacobianTimeVariation(model,data,jointId,se3.ReferenceFrame.LOCAL)
  else:
    return se3.getJointJacobianTimeVariation(model,data,jointId,se3.ReferenceFrame.WORLD)

@deprecated("This function has been renamed difference and will be removed in release 1.4.0 of Pinocchio. Please change for new difference.")
def differentiate(model,q0,q1):
  return se3.difference(model,q0,q1)
