#
# Copyright (c) 2018-2020 CNRS INRIA
#

## In this file, are reported some deprecated functions that are still maintained until the next important future releases ##

from __future__ import print_function

import warnings as _warnings

from . import pinocchio_pywrap as pin
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

class GeometryObject(pin.GeometryObject):
    @property
    @deprecated("The fcl property has been renamed geometry. Please use GeometryObject.geometry instead")
    def fcl(self):
       return self.geometry

@deprecated("This function is now called SE3ToXYZQUATtuple. Please change for this new signature to delete this warning.")
def se3ToXYZQUATtuple(M):
    return pin.SE3ToXYZQUATtuple(M)

@deprecated("This function is now called SE3ToXYZQUAT. Please change for this new signature to delete this warning.")
def se3ToXYZQUAT(M):
    return pin.SE3ToXYZQUAT(M)

@deprecated("This function is now called XYZQUATToSE3. Please change for this new signature to delete this warning.")
def XYZQUATToSe3(x):
    return pin.XYZQUATToSE3(x)

def buildGeomFromUrdf(model, filename, *args):

  arg3 = args[0]
  if isinstance(arg3,(str,list,pin.StdVec_StdString)):
    package_dir = arg3
    geom_type = args[1]

    if len(args) >= 3:
      mesh_loader = args[2]
      message = ("This function signature is now deprecated and will be removed in future releases of Pinocchio. "
                 "Please change for the new signature buildGeomFromUrdf(model,filename,type,package_dirs,mesh_loader).")
      _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
      return pin.buildGeomFromUrdf(model,filename,geom_type,package_dir,mesh_loader)
    else:
      message = ("This function signature is now deprecated and will be removed in future releases of Pinocchio. "
                 "Please change for the new signature buildGeomFromUrdf(model,filename,type,package_dirs).")
      _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
      return pin.buildGeomFromUrdf(model,filename,geom_type,package_dir)
  else:
    return pin.buildGeomFromUrdf(model, filename, *args)

buildGeomFromUrdf.__doc__ = (
  pin.buildGeomFromUrdf.__doc__
)

@deprecated("This function is now deprecated and will be removed in future releases of Pinocchio. "
            "Please change for the new function computePotentialEnergy.")
def potentialEnergy(model,data,q,update_kinematics=True):
  if update_kinematics:
    return pin.computePotentialEnergy(model,data,q)
  else:
    return pin.computePotentialEnergy(model,data)

potentialEnergy.__doc__ += '\n' + pin.computePotentialEnergy.__doc__

@deprecated("This function is now deprecated and will be removed in future releases of Pinocchio. "
            "Please change for the new function computeKineticEnergy.")
def kineticEnergy(model,data,q,v,update_kinematics=True):
  if update_kinematics:
    return pin.computeKineticEnergy(model,data,q,v)
  else:
    return pin.computeKineticEnergy(model,data)

kineticEnergy.__doc__ += '\n' + pin.computeKineticEnergy.__doc__

from .utils import npToTTuple, npToTuple
pin.rpy.npToTTuple = deprecated("This function was moved to the utils submodule.")(npToTTuple)
pin.rpy.npToTuple = deprecated("This function was moved to the utils submodule.")(npToTuple)

# Marked as deprecated on 26 Mar 2020
@deprecated("This function is now deprecated without replacement.")
def setGeometryMeshScales(geom_model, mesh_scale):
  import numpy as np
  if not isinstance(mesh_scale, np.ndarray):
    mesh_scale = np.array([mesh_scale]*3)
  for geom in geom_model.geometryObjects:
    geom.meshScale = mesh_scale
