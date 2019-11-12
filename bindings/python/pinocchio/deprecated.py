#
# Copyright (c) 2018-2019 CNRS INRIA
#

## In this file, are reported some deprecated functions that are still maintained until the next important future releases ##

from __future__ import print_function

import warnings as _warnings

from . import libpinocchio_pywrap as pin 
from .deprecation import deprecated, DeprecatedWarning

# This function is only deprecated when using a specific signature. Therefore, it needs special care
# Marked as deprecated on 16 Sept 2019
def impulseDynamics(model, data, *args):
  if len(args)==5 and type(args[4]) is bool:
    message = ("This function signature has been deprecated and will be removed in future releases of Pinocchio. "
               "Please change for the new signature of impulseDynamics(model,data[,q],v_before,J[,r_coeff[,inv_damping]]).")
    _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
    q = args[0]
    v_before = args[1]
    J = args[2]
    r_coeff = args[3]
    updateKinematics = args[4]
    inv_damping = 0.
    if updateKinematics:
      return pin.impulseDynamics(model,data,q,v_before,J,r_coeff,inv_damping)
    else:
      return pin.impulseDynamics(model,data,v_before,J,r_coeff,inv_damping)

  return pin.impulseDynamics(model, data, *args)

impulseDynamics.__doc__ =  (
  pin.impulseDynamics.__doc__
  + '\n\nimpulseDynamics( (Model)Model, (Data)Data, (object)Joint configuration q (size Model::nq), (object)Joint velocity before impact v_before (size Model::nv), (object)Contact Jacobian J (size nb_constraint * Model::nv), (float)Coefficient of restitution r_coeff (0 = rigid impact; 1 = fully elastic impact), (bool)updateKinematics) -> object :'
  + '\n    This function signature has been deprecated and will be removed in future releases of Pinocchio.'
)

# This function is only deprecated when using a specific signature. Therefore, it needs special care
# Marked as deprecated on 2 Oct 2019
def forwardDynamics(model, data, *args):
  if len(args)==7 and type(args[6]) is bool:
    message = ("This function signature has been deprecated and will be removed in future releases of Pinocchio. "
               "Please change for the new signature of forwardDynamics(model,data[,q],v,tau,J,gamma[,inv_damping]).")
    _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
    q = args[0]
    v = args[1]
    tau = args[2]
    J = args[3]
    gamma = args[4]
    inv_damping = args[5]
    updateKinematics = args[6]
    if updateKinematics:
      return pin.forwardDynamics(model,data,q,v,tau,J,gamma,inv_damping)
    else:
      return pin.forwardDynamics(model,data,tau,J,gamma,inv_damping)

  return pin.forwardDynamics(model, data, *args)

forwardDynamics.__doc__ = (
  pin.forwardDynamics.__doc__
  + '\n\nforwardDynamics( (Model)Model, (Data)Data, (object)Joint configuration q (size Model::nq), (object)Joint velocity v (size Model::nv), (object)Joint torque tau (size Model::nv), (object)Contact Jacobian J (size nb_constraint * Model::nv), (object)Contact drift gamma (size nb_constraint), (float)(double) Damping factor for cholesky decomposition of JMinvJt. Set to zero if constraints are full rank, (bool)Update kinematics) -> object :'
+ '\n    This function signature has been deprecated and will be removed in future releases of Pinocchio.'
)

@deprecated("This function has been renamed computeJointJacobian and will be removed in future releases of Pinocchio. Please change for new computeJointJacobian.")
def jointJacobian(model, data, q, jointId):
  return pin.computeJointJacobian(model,data,q,jointId)
  
@deprecated("This function has been renamed computeFrameJacobian and will be removed in future releases of Pinocchio. Please change for new computeFrameJacobian.")
def frameJacobian(model, data, q, frameId):
  return pin.computeFrameJacobian(model,data,q,frameId)
  
def computeCentroidalDynamics(model, data, q, v, a = None):
  if a is None:
    message = ("This function signature has been renamed and will be removed in future releases of Pinocchio. "
               "Please change for the new signature of computeCentroidalMomentum(model,data,q,v).")
    _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
    return pin.computeCentroidalMomentum(model,data,q,v)
  else:
    message = ("This function signature has been renamed and will be removed in future releases of Pinocchio. "
               "Please change for the new signature of computeCentroidalMomentumTimeVariation(model,data,q,v,a).")
    _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
    return pin.computeCentroidalMomentum(model,data,q,v,a)

computeCentroidalDynamics.__doc__ = ( "This function has been renamed computeCentroidalMomentum or computeCentroidalMomentumTimeVariation to either only compute the centroidal momentum quantity or also its time derivative respectively." )
