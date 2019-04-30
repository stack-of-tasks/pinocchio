#
# Copyright (c) 2018 CNRS
#

## In this file, some shortcuts are provided ##

from . import libpinocchio_pywrap as pin

nle = pin.nonLinearEffects

def _buildGeomFromUrdf (model, filename, geometryType, meshLoader, dirs=None):
    """Helper function. It ignores meshLoader if Pinocchio was compiled without hpp-fcl."""
    if meshLoader is None or not pin.WITH_FCL_SUPPORT():
        if dirs:
            return pin.buildGeomFromUrdf(model, filename, dirs, geometryType)
        else:
            return pin.buildGeomFromUrdf(model, filename, geometryType)
    else:
        if dirs:
            return pin.buildGeomFromUrdf(model, filename, dirs, geometryType, meshLoader)
        else:
            return pin.buildGeomFromUrdf(model, filename, geometryType, meshLoader)

def buildModelsFromUrdf(filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None):
    """Parse the URDF file given in input and return model, collision model, and visual model (in this order)"""
    if root_joint is None:
        model = pin.buildModelFromUrdf(filename)
    else:
        model = pin.buildModelFromUrdf(filename, root_joint)

    if "buildGeomFromUrdf" not in dir(pin):
        collision_model = None
        visual_model = None
        if verbose:
            print('Info: the Geometry Module has not been compiled with Pinocchio. No geometry model and data have been built.')
    else:
        if verbose and "removeCollisionPairs" not in dir(pin) and meshLoader is not None:
            print('Info: Pinocchio was compiled without hpp-fcl. meshLoader is ignored.')

        if package_dirs is None:
            collision_model = _buildGeomFromUrdf(model, filename, pin.GeometryType.COLLISION,meshLoader)
            visual_model = _buildGeomFromUrdf(model, filename, pin.GeometryType.VISUAL, meshLoader)
        else:
            if not all(isinstance(item, str) for item in package_dirs):
                raise Exception('The list of package directories is wrong. At least one is not a string')
            else:
                collision_model = _buildGeomFromUrdf(model, filename, pin.GeometryType.COLLISION, meshLoader, package_dirs)
                visual_model = _buildGeomFromUrdf(model, filename, pin.GeometryType.VISUAL, meshLoader, package_dirs)

    return model, collision_model, visual_model