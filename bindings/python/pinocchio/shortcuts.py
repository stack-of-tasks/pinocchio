#
# Copyright (c) 2018 CNRS
#

## In this file, some shortcuts are provided ##

from . import libpinocchio_pywrap as pin

nle = pin.nonLinearEffects

def buildModelsFromUrdf(filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None, geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL]):
    """Parse the URDF file given in input and return a Pinocchio Model followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
    Examples of usage:
        # load model, collision model, and visual model, in this order (default)
        model, collision_model, visual_model = buildModelsFromUrdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        model, collision_model, visual_model = buildModelsFromUrdf(filename[, ...]) # same as above
        
        model, collision_model = buildModelsFromUrdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION]) # only load the model and the collision model
        model, collision_model = buildModelsFromUrdf(filename[, ...], geometry_types=pin.GeometryType.COLLISION)   # same as above
        model, visual_model    = buildModelsFromUrdf(filename[, ...], geometry_types=pin.GeometryType.VISUAL)      # only load the model and the visual model
        
        model = buildModelsFromUrdf(filename[, ...], geometry_types=[])  # equivalent to buildModelFromUrdf(filename[, root_joint])
    """

    if root_joint is None:
        model = pin.buildModelFromUrdf(filename)
    else:
        model = pin.buildModelFromUrdf(filename, root_joint)

    if verbose and not pin.WITH_FCL_SUPPORT() and meshLoader is not None:
        print('Info: Pinocchio was compiled without hpp-fcl. meshLoader is ignored.')
    if package_dirs is None:
        package_dirs = []

    lst = [model]

    if not hasattr(geometry_types, '__iter__'):
        geometry_types = [geometry_types]

    for geometry_type in geometry_types:
        if meshLoader is None or not pin.WITH_FCL_SUPPORT():
            geom_model = pin.buildGeomFromUrdf(model, filename, package_dirs, geometry_type)
        else:
            geom_model = pin.buildGeomFromUrdf(model, filename, package_dirs, geometry_type, meshLoader)
        lst.append(geom_model)

    return tuple(lst)

def createDatas(*models):
    """Call createData() on each Model or GeometryModel in input and return the results in a tuple.
    If one of the models is None, the corresponding data object in the result is also None.
    """
    return tuple([None if model is None else model.createData() for model in models])
