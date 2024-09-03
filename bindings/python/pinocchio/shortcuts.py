# ruff: noqa: E501
#
# Copyright (c) 2018-2020 CNRS INRIA
#

## In this file, some shortcuts are provided ##

from typing import Tuple

from . import WITH_HPP_FCL, WITH_HPP_FCL_BINDINGS
from . import pinocchio_pywrap_default as pin

nle = pin.nonLinearEffects


def buildModelsFromUrdf(
    filename,
    package_dirs=None,
    root_joint=None,
    root_joint_name=None,
    verbose=False,
    meshLoader=None,
    geometry_types=[pin.GeometryType.COLLISION, pin.GeometryType.VISUAL],
) -> Tuple[pin.Model, pin.GeometryModel, pin.GeometryModel]:
    """Parse the URDF file given in input and return a Pinocchio Model followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
    Examples of usage:
        # load model, collision model, and visual model, in this order (default)
        model, collision_model, visual_model = buildModelsFromUrdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        model, collision_model, visual_model = buildModelsFromUrdf(filename[, ...]) # same as above

        model, collision_model = buildModelsFromUrdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION]) # only load the model and the collision model
        model, collision_model = buildModelsFromUrdf(filename[, ...], geometry_types=pin.GeometryType.COLLISION)   # same as above
        model, visual_model    = buildModelsFromUrdf(filename[, ...], geometry_types=pin.GeometryType.VISUAL)      # only load the model and the visual model

        model = buildModelsFromUrdf(filename[, ...], geometry_types=[])  # equivalent to buildModelFromUrdf(filename[, root_joint])

    Remark:
        Remark: In the URDF format, a joint of type fixed can be defined.
        For efficiency reasons, it is treated as operational frame and not as a joint of the model.
    """
    if geometry_types is None:
        geometry_types = [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]
    if root_joint is None:
        model = pin.buildModelFromUrdf(filename)
    else:
        model = pin.buildModelFromUrdf(filename, root_joint, root_joint_name)

    if verbose and not WITH_HPP_FCL and meshLoader is not None:
        print(
            "Info: MeshLoader is ignored. Pinocchio has not been compiled with HPP-FCL."
        )
    if verbose and not WITH_HPP_FCL_BINDINGS and meshLoader is not None:
        print(
            "Info: MeshLoader is ignored. The HPP-FCL Python bindings have not been installed."
        )
    if package_dirs is None:
        package_dirs = []

    lst = [model]

    if not hasattr(geometry_types, "__iter__"):
        geometry_types = [geometry_types]

    for geometry_type in geometry_types:
        if meshLoader is None or (not WITH_HPP_FCL and not WITH_HPP_FCL_BINDINGS):
            geom_model = pin.buildGeomFromUrdf(
                model, filename, geometry_type, package_dirs=package_dirs
            )
        else:
            geom_model = pin.buildGeomFromUrdf(
                model,
                filename,
                geometry_type,
                package_dirs=package_dirs,
                mesh_loader=meshLoader,
            )
        lst.append(geom_model)

    return tuple(lst)


def createDatas(*models):
    """
    Call createData() on each Model or GeometryModel in input and return the results in
    a tuple. If one of the models is None, the corresponding data object in the result
    is also None.
    """
    return tuple([None if model is None else model.createData() for model in models])


def buildModelsFromSdf(
    filename,
    package_dirs=None,
    root_joint=None,
    root_joint_name=None,
    root_link_name="",
    parent_guidance=[],
    verbose=False,
    meshLoader=None,
    geometry_types=None,
):
    """Parse the SDF file given in input and return a Pinocchio Model and a list of Constraint Models, followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
    Examples of usage:
        # load model, constraint models, collision model, and visual model, in this order (default)
        model, constraint_models, collision_model, visual_model = buildModelsFromSdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        model, constraint_models, collision_model, visual_model = buildModelsFromSdf(filename[, ...]) # same as above
        model, constraint_models, collision_model = buildModelsFromSdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION]) # only load the model, constraint models and the collision model
        model, constraint_models, collision_model = buildModelsFromSdf(filename[, ...], geometry_types=pin.GeometryType.COLLISION)   # same as above
        model, constraint_models, visual_model    = buildModelsFromSdf(filename[, ...], geometry_types=pin.GeometryType.VISUAL)      # only load the model and the visual model
        model, constraint_models = buildModelsFromSdf(filename[, ...], geometry_types=[])  # equivalent to buildModelFromSdf(filename[, root_joint])
    """
    if geometry_types is None:
        geometry_types = [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]
    if root_joint is None:
        model, constraint_models = pin.buildModelFromSdf(
            filename, root_link_name, parent_guidance
        )
    else:
        model, constraint_models = pin.buildModelFromSdf(
            filename, root_joint, root_link_name, root_joint_name, parent_guidance
        )

    if verbose and not WITH_HPP_FCL and meshLoader is not None:
        print(
            "Info: MeshLoader is ignored. Pinocchio has not been compiled with HPP-FCL."
        )
    if verbose and not WITH_HPP_FCL_BINDINGS and meshLoader is not None:
        print(
            "Info: MeshLoader is ignored. "
            "The HPP-FCL Python bindings have not been installed."
        )
    if package_dirs is None:
        package_dirs = []

    lst = [model, constraint_models]

    if not hasattr(geometry_types, "__iter__"):
        geometry_types = [geometry_types]

    for geometry_type in geometry_types:
        if meshLoader is None or (not WITH_HPP_FCL and not WITH_HPP_FCL_BINDINGS):
            geom_model = pin.buildGeomFromSdf(
                model, filename, geometry_type, root_link_name, package_dirs
            )
        else:
            geom_model = pin.buildGeomFromSdf(
                model, filename, geometry_type, root_link_name, package_dirs, meshLoader
            )
        lst.append(geom_model)

    return tuple(lst)


def buildModelsFromMJCF(
    filename,
    root_joint=None,
    root_joint_name=None,
    verbose=False,
    meshLoader=None,
    geometry_types=None,
):
    """Parse the Mjcf file given in input and return a Pinocchio Model, followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
    Examples of usage:
        # load model, constraint models, collision model, and visual model, in this order (default)
        # load model, collision model, and visual model, in this order (default)
        model, collision_model, visual_model = buildModelsFromMJCF(filename[, ...], geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
        model, collision_model, visual_model = buildModelsFromMJCF(filename[, ...]) # same as above

        model, collision_model = buildModelsFromMJCF(filename[, ...], geometry_types=[pin.GeometryType.COLLISION]) # only load the model and the collision model
        model, collision_model = buildModelsFromMJCF(filename[, ...], geometry_types=pin.GeometryType.COLLISION)   # same as above
        model, visual_model    = buildModelsFromMJCF(filename[, ...], geometry_types=pin.GeometryType.VISUAL)      # only load the model and the visual model

        model = buildModelsFromMJCF(filename[, ...], geometry_types=[])  # equivalent to buildModelFromMJCF(filename[, root_joint])
    """
    if geometry_types is None:
        geometry_types = [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]
    if root_joint is None:
        model = pin.buildModelFromMJCF(filename)
    else:
        model = pin.buildModelFromMJCF(filename, root_joint, root_joint_name)

    if verbose and not WITH_HPP_FCL and meshLoader is not None:
        print(
            "Info: MeshLoader is ignored. Pinocchio has not been compiled with HPP-FCL."
        )
    if verbose and not WITH_HPP_FCL_BINDINGS and meshLoader is not None:
        print(
            "Info: MeshLoader is ignored. "
            "The HPP-FCL Python bindings have not been installed."
        )

    lst = [model]

    if not hasattr(geometry_types, "__iter__"):
        geometry_types = [geometry_types]

    for geometry_type in geometry_types:
        if meshLoader is None or (not WITH_HPP_FCL and not WITH_HPP_FCL_BINDINGS):
            geom_model = pin.buildGeomFromMJCF(model, filename, geometry_type)
        else:
            geom_model = pin.buildGeomFromMJCF(
                model, filename, geometry_type, mesh_loader=meshLoader
            )
        lst.append(geom_model)

    return tuple(lst)
