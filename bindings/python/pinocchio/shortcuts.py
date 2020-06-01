#
# Copyright (c) 2018-2020 CNRS INRIA
#

## In this file, some shortcuts are provided ##

from . import pinocchio_pywrap as pin
from . import WITH_HPP_FCL, WITH_HPP_FCL_BINDINGS

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

    if verbose and not WITH_HPP_FCL and meshLoader is not None:
        print('Info: MeshLoader is ignored. Pinocchio has not been compiled with HPP-FCL.')
    if verbose and not WITH_HPP_FCL_BINDINGS and meshLoader is not None:
        print('Info: MeshLoader is ignored. The HPP-FCL Python bindings have not been installed.')
    if package_dirs is None:
        package_dirs = []

    lst = [model]

    if not hasattr(geometry_types, '__iter__'):
        geometry_types = [geometry_types]

    for geometry_type in geometry_types:
        if meshLoader is None or (not WITH_HPP_FCL and not WITH_HPP_FCL_BINDINGS):
            geom_model = pin.buildGeomFromUrdf(model, filename, geometry_type, package_dirs)
        else:
            geom_model = pin.buildGeomFromUrdf(model, filename, geometry_type, package_dirs, meshLoader)
        lst.append(geom_model)

    return tuple(lst)

def createDatas(*models):
    """Call createData() on each Model or GeometryModel in input and return the results in a tuple.
    If one of the models is None, the corresponding data object in the result is also None.
    """
    return tuple([None if model is None else model.createData() for model in models])
