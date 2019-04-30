#
# Copyright (c) 2018 CNRS
#

## In this file, some shortcuts are provided ##

from . import libpinocchio_pywrap as pin

nle = pin.nonLinearEffects

def _buildGeomFromUrdf (model, filename, dirs, geometryType, meshLoader):
    """Helper function. It ignores meshLoader if Pinocchio was compiled without hpp-fcl."""
    if meshLoader is None or not pin.WITH_FCL_SUPPORT():
        return pin.buildGeomFromUrdf(model, filename, dirs, geometryType)
    else:
        return pin.buildGeomFromUrdf(model, filename, dirs, geometryType, meshLoader)

def buildModelsFromUrdf(filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None):
    """Parse the URDF file given in input and return model, collision model, and visual model (in this order)"""
    if root_joint is None:
        model = pin.buildModelFromUrdf(filename)
    else:
        model = pin.buildModelFromUrdf(filename, root_joint)

    if verbose and not pin.WITH_FCL_SUPPORT() and meshLoader is not None:
        print('Info: Pinocchio was compiled without hpp-fcl. meshLoader is ignored.')
    if package_dirs is None:
        package_dirs = []

    collision_model = _buildGeomFromUrdf(model, filename, package_dirs, pin.GeometryType.COLLISION, meshLoader)
    visual_model = _buildGeomFromUrdf(model, filename, package_dirs, pin.GeometryType.VISUAL, meshLoader)

    return model, collision_model, visual_model