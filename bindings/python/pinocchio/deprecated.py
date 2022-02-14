#
# Copyright (c) 2018-2021 CNRS INRIA
#

## In this file, are reported some deprecated functions that are still maintained until the next important future releases ##

from __future__ import print_function

import warnings as _warnings

from . import pinocchio_pywrap_default as pin
from .deprecation import deprecated, DeprecatedWarning

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

def buildGeomFromUrdf(model, filename, *args, **kwargs):

  arg3 = args[0]
  if isinstance(arg3,(str,list,pin.StdVec_StdString)):
    package_dir = arg3
    geom_type = args[1]

    if len(args) >= 3:
      mesh_loader = args[2]
      message = ("This function signature is now deprecated and will be removed in future releases of Pinocchio. "
                 "Please change for the new signature buildGeomFromUrdf(model,filename,type,package_dirs,mesh_loader).")
      _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
      return pin.buildGeomFromUrdf(model,filename,geom_type,package_dir,mesh_loader, **kwargs)
    else:
      message = ("This function signature is now deprecated and will be removed in future releases of Pinocchio. "
                 "Please change for the new signature buildGeomFromUrdf(model,filename,type,package_dirs).")
      _warnings.warn(message, category=DeprecatedWarning, stacklevel=2)
      return pin.buildGeomFromUrdf(model,filename,geom_type,package_dir, **kwargs)
  else:
    return pin.buildGeomFromUrdf(model, filename, *args, **kwargs)

buildGeomFromUrdf.__doc__ = (
  pin.buildGeomFromUrdf.__doc__
)

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
