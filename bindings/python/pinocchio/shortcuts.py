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

def buildModelsFromUrdf(filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None, geometry_type=None):
    """Parse the URDF file given in input and return a Pinocchio Model and appropriate GeometryModel(s).
    You can use this function in two ways.
    
    The first one is by specifying the geometry type, as in
        model, collision_model = buildModelsFromUrdf(filename[, ...], geometry_type=pin.GeometryType.COLLISION)
        model, visual_model    = buildModelsFromUrdf(filename[, ...], geometry_type=pin.GeometryType.VISUAL)
    
    In this case, the function will return a Pinocchio Model and a GeometryModel of the specified type.
    
    The second one is by leaving the geometry type unspecified (i.e. None), for instance as in
        model, collision_model, visual_model = buildModelsFromUrdf(filename)
    
    In this case, the function will return a Pinocchio Model, a collision model, and visual model (in this order)
    """

    if root_joint is None:
        model = pin.buildModelFromUrdf(filename)
    else:
        model = pin.buildModelFromUrdf(filename, root_joint)

    if verbose and not pin.WITH_FCL_SUPPORT() and meshLoader is not None:
        print('Info: Pinocchio was compiled without hpp-fcl. meshLoader is ignored.')
    if package_dirs is None:
        package_dirs = []

    if geometry_type is None:
        collision_model = _buildGeomFromUrdf(model, filename, package_dirs, pin.GeometryType.COLLISION, meshLoader)
        visual_model = _buildGeomFromUrdf(model, filename, package_dirs, pin.GeometryType.VISUAL, meshLoader)
        return model, collision_model, visual_model
    else:
        geom_model = _buildGeomFromUrdf(model, filename, package_dirs, geometry_type, meshLoader)
        return model, geom_model
