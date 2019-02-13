#
# Copyright (c) 2018 CNRS INRIA
#

## In this file, are reported some deprecated functions that are still maintained until the next important future releases ##

from __future__ import print_function

from . import libpinocchio_pywrap as pin 
from .deprecation import deprecated, DeprecatedWarning

@deprecated("This function has been renamed to impulseDynamics for consistency with the C++ interface. Please change for impulseDynamics.")
def impactDynamics(model, data, q, v_before, J, r_coeff=0.0, update_kinematics=True):
  return pin.impulseDynamics(model, data, q, v_before, J, r_coeff, update_kinematics)

@deprecated("This function has been deprecated. Please use buildSampleModelHumanoid or buildSampleModelHumanoidRandom instead.")
def buildSampleModelHumanoidSimple(usingFF=True):
    return pin.buildSampleModelHumanoidRandom(usingFF)

@deprecated("Static method Model.BuildHumanoidSimple has been deprecated. Please use function buildSampleModelHumanoid or buildSampleModelHumanoidRandom instead.")
def _Model__BuildHumanoidSimple(usingFF=True):
    return pin.buildSampleModelHumanoidRandom(usingFF)

pin.Model.BuildHumanoidSimple = staticmethod(_Model__BuildHumanoidSimple)

@deprecated("Static method Model.BuildEmptyModel has been deprecated. Please use the empty Model constructor instead.")
def _Model__BuildEmptyModel():
    return pin.Model()

pin.Model.BuildEmptyModel = staticmethod(_Model__BuildEmptyModel)

@deprecated("This function has been renamed updateFramePlacements when taking two arguments, and framesForwardKinematics when taking three. Please change your code to the appropriate method.")
def framesKinematics(model,data,q=None):
  if q is None:
    pin.updateFramePlacements(model,data)
  else:
    pin.framesForwardKinematics(model,data,q)

@deprecated("This function has been renamed computeJointJacobians and will be removed in future releases of Pinocchio. Please change for new computeJointJacobians.")
def computeJacobians(model,data,q=None):
  if q is None:
    return pin.computeJointJacobians(model,data)
  else:
    return pin.computeJointJacobians(model,data,q)

@deprecated("This function has been renamed jointJacobian and will be removed in future releases of Pinocchio. Please change for new jointJacobian function.")
def jacobian(model,data,q,jointId,local,update_kinematics):
  if local:
    return pin.jointJacobian(model,data,q,jointId,pin.ReferenceFrame.LOCAL,update_kinematics)
  else:
    return pin.jointJacobian(model,data,q,jointId,pin.ReferenceFrame.WORLD,update_kinematics)

@deprecated("This function has been renamed getJointJacobian and will be removed in future releases of Pinocchio. Please change for new getJointJacobian function.")
def getJacobian(model,data,jointId,local):
  if local:
    return pin.getJointJacobian(model,data,jointId,pin.ReferenceFrame.LOCAL)
  else:
    return pin.getJointJacobian(model,data,jointId,pin.ReferenceFrame.WORLD)

@deprecated("This function has been renamed computeJacobiansTimeVariation and will be removed in future releases of Pinocchio. Please change for new computeJointJacobiansTimeVariation.")
def computeJacobiansTimeVariation(model,data,q,v):
  return pin.computeJointJacobiansTimeVariation(model,data,q,v)

@deprecated("This function has been renamed getJointJacobianTimeVariation and will be removed in future releases of Pinocchio. Please change for new getJointJacobianTimeVariation function.")
def getJacobianTimeVariation(model,data,jointId,local):
  if local:
    return pin.getJointJacobianTimeVariation(model,data,jointId,pin.ReferenceFrame.LOCAL)
  else:
    return pin.getJointJacobianTimeVariation(model,data,jointId,pin.ReferenceFrame.WORLD)

@deprecated("This function has been renamed difference and will be removed in future releases of Pinocchio. Please change for new difference function.")
def differentiate(model,q0,q1):
  return pin.difference(model,q0,q1)

@deprecated("This function has been renamed loadReferenceConfigurations and will be removed in future releases of Pinocchio. Please change for new loadReferenceConfigurations function.")
def getNeutralConfigurationFromSrdf(model, filename, verbose):
  pin.loadReferenceConfigurations(model,filename,verbose)
  return model.referenceConfigurations["half_sitting"]

@deprecated("This function has been renamed loadReferenceConfigurations and will be removed in future releases of Pinocchio. Please change for new loadReferenceConfigurations function.")
def getNeutralConfiguration(model, filename, verbose):
  pin.loadReferenceConfigurations(model,filename,verbose)
  return model.referenceConfigurations["half_sitting"]

@deprecated("This function has been renamed difference and will be removed in future releases of Pinocchio. Please change for new loadRotorParameters function.")
def loadRotorParamsFromSrdf(model, filename, verbose):
  return pin.loadRotorParams(model,filename,verbose)

@deprecated("This function has been renamed difference and will be removed in future releases of Pinocchio. Please change for new removeCollisionPairs function.")
def removeCollisionPairsFromSrdf(model, geomModel, filename, verbose):
  return pin.removeCollisionPairs(model,geomModel,filename,verbose)

# This function is only deprecated when using a specific signature. Therefore, it needs special care
def jacobianCenterOfMass(model, data, *args):
  if len(args)==3:
    import warnings
    import inspect
    message = "This function signature has been deprecated and will be removed in future releases of Pinocchio. Please change for one of the new signatures of the jacobianCenterOfMass function."
    frame = inspect.currentframe().f_back
    warnings.warn_explicit(message,
                           category=DeprecatedWarning,
                           filename=inspect.getfile(frame.f_code),
                           lineno=frame.f_lineno)
    q = args[0]
    computeSubtreeComs = args[1]
    updateKinematics = args[2]
    if updateKinematics:
      return pin.jacobianCenterOfMass(model,data,q,computeSubtreeComs)
    else:
      return pin.jacobianCenterOfMass(model,data,computeSubtreeComs)
  else:
    return pin.jacobianCenterOfMass(model,data,*args)
jacobianCenterOfMass.__doc__ =  (
  pin.jacobianCenterOfMass.__doc__
  + '\n\njacobianCenterOfMass( (Model)Model, (Data)Data, (object)Joint configuration q (size Model::nq), (bool)computeSubtreeComs, (bool)updateKinematics) -> object :'
  + '\n    This function signature has been deprecated and will be removed in future releases of Pinocchio.'
)

@deprecated("This function will be removed in future releases of Pinocchio. You can use exp or exp6.")
def exp6FromMotion(motion):
  return pin.exp6(motion)

@deprecated("This function will be removed in future releases of Pinocchio. You can build a Motion object from a 6D vector and use the standard exp function to recover the same behavior.")
def exp6FromVector(vector6):
  v = pin.Motion(vector6)
  return pin.exp6(v)

@deprecated("This function will be removed in future releases of Pinocchio. You can use log or log6.")
def log6FromSE3(transform):
  return pin.log6(transform)

@deprecated("This function will be removed in future releases of Pinocchio. You can use log or log6.")
def log6FromMatrix(matrix4):
  return pin.log6(matrix4)
