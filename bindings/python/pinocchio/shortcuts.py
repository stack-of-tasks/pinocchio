# ruff: noqa: E501
#
# Copyright (c) 2018-2020 CNRS INRIA
#

## In this file, some shortcuts are provided ##

from . import WITH_HPP_FCL, WITH_HPP_FCL_BINDINGS
from . import pinocchio_pywrap_default as pin

nle = pin.nonLinearEffects


def buildModelsFromUrdf(
    filename, *args, **kwargs
) -> tuple[pin.Model, pin.GeometryModel, pin.GeometryModel]:
    """Parse the URDF file given in input and return a Pinocchio Model followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
    Arguments:
        - filename - name of the urdf file to load
        - package_dirs - where the meshes of the urdf are located. (default - None)
        - root_joint - Joint at the base of the model (default - None)
        - root_joint_name - Name for the root_joint (default - "root_joint")
        - verbose - print information of parsing (default - False)
        - meshLoader - object used to load meshes (default - hpp::fcl::MeshLoader)
        - geometry_types - Which geometry model to load. Can be pin.GeometryType.COLLISION, pin.GeometryType.VISUAL or both. (default - [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL])
    Return:
        Tuple of the models, in this order : model, collision model, and visual model.

    Example:
        model, collision_model, visual_model = buildModelsFromUrdf(filename, root_joint, verbose, meshLoader, geometry_types, root_joint_name="root_joint_name")

    Remark: In the URDF format, a joint of type fixed can be defined. For efficiency reasons, it is treated as operational frame and not as a joint of the model.
    """
    # Handle the switch from old to new api
    arg_keys = ["package_dirs", "root_joint", "verbose", "meshLoader", "geometry_types"]
    if len(args) >= 3:
        if isinstance(args[2], str):
            arg_keys = [
                "package_dirs",
                "root_joint",
                "root_joint_name",
                "verbose",
                "meshLoader",
                "geometry_types",
            ]

    for key, arg in zip(arg_keys, args):
        if key in kwargs.keys():
            raise TypeError("Function got multiple values for argument ", key)
        else:
            kwargs[key] = arg

    return _buildModelsFromUrdf(filename, **kwargs)


def _buildModelsFromUrdf(
    filename,
    package_dirs=None,
    root_joint=None,
    root_joint_name=None,
    verbose=False,
    meshLoader=None,
    geometry_types=None,
) -> tuple[pin.Model, pin.GeometryModel, pin.GeometryModel]:
    if geometry_types is None:
        geometry_types = [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]

    if root_joint is None:
        model = pin.buildModelFromUrdf(filename)
    elif root_joint is not None and root_joint_name is None:
        model = pin.buildModelFromUrdf(filename, root_joint)
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
    filename, *args, **kwargs
) -> tuple[pin.Model, pin.GeometryModel, pin.GeometryModel]:
    """Parse the Sdf file given in input and return a Pinocchio Model and a list of Constraint Models, followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
    Arguments:
        - filename - name of the urdf file to load
        - package_dirs - where the meshes of the urdf are located. (default - None)
        - root_joint - Joint at the base of the model (default - None)
        - root_link_name - Name of the body to use as root of the model (default - "")
        - root_joint_name - Name for the root_joint (default - "root_joint")
        - parent_guidance - Joint names which should be preferred for cases where two joints can qualify as parent. The other joint appears in the constraint_model. If empty, joint appearance order in .sdf is taken as default.
        - verbose - print information of parsing (default - False)
        - meshLoader - object used to load meshes (default - hpp::fcl::MeshLoader)
        - geometry_types - Which geometry model to load. Can be pin.GeometryType.COLLISION, pin.GeometryType.VISUAL, both or None. (default - None])
    Return:
        Tuple of the models, in this order : model, collision model, and visual model.

    Example:
        model, collision_model, visual_model = buildModelsFromSdf(filename, root_joint, root_link_name, parent_guidance, verbose, meshLoader, geometry_types, root_joint_name="root_joint_name")
    """

    arg_keys = [
        "package_dirs",
        "root_joint",
        "root_link_name",
        "parent_guidance",
        "verbose",
        "meshLoader",
        "geometry_types",
    ]
    # Handle the switch from old to new api
    if len(args) >= 4:
        if isinstance(args[3], str):
            arg_keys = [
                "package_dirs",
                "root_joint",
                "root_link_name",
                "root_joint_name",
                "parent_guidance",
                "verbose",
                "meshLoader",
                "geometry_types",
            ]

    for key, arg in zip(arg_keys, args):
        if key in kwargs.keys():
            raise TypeError("Function got multiple values for argument ", key)
        else:
            kwargs[key] = arg

    return _buildModelsFromSdf(filename, **kwargs)


def _buildModelsFromSdf(
    filename,
    package_dirs=None,
    root_joint=None,
    root_link_name="",
    root_joint_name=None,
    parent_guidance=None,
    verbose=False,
    meshLoader=None,
    geometry_types=None,
):
    if parent_guidance is None:
        parent_guidance = []

    if geometry_types is None:
        geometry_types = [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]

    if root_joint is None:
        model, constraint_models = pin.buildModelFromSdf(
            filename, root_link_name, parent_guidance
        )
    elif root_joint is not None and root_joint_name is None:
        model, constraint_models = pin.buildModelFromSdf(
            filename, root_joint, root_link_name, parent_guidance
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
            "Info: MeshLoader is ignored. The HPP-FCL Python bindings have not been installed."
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


def buildModelsFromMJCF(filename, *args, **kwargs):
    """Parse the Mjcf file given in input and return a Pinocchio Model followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
    Arguments:
        - filename - name of the urdf file to load
        - package_dirs - where the meshes of the urdf are located. (default - None)
        - root_joint - Joint at the base of the model (default - None)
        - root_joint_name - Name for the root_joint (default - "root_joint")
        - verbose - print information of parsing (default - False)
        - meshLoader - object used to load meshes (default - hpp::fcl::MeshLoader)
        - geometry_types - Which geometry model to load. Can be pin.GeometryType.COLLISION, pin.GeometryType.VISUAL or both. (default - [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL])
        - contacts - Boolean to know if contraint models are wanted (default - False)
    Return:
        Tuple of the models, in this order : model, collision model, and visual model, or  model, constraint_list, collision model, and visual model, if contacts is True.

    Example:
        model, collision_model, visual_model = buildModelsFromMJCF(filename, root_joint, verbose, meshLoader, geometry_types, root_joint_name="root_joint_name")
    """
    # Handle the switch from old to new api
    arg_keys = ["root_joint", "verbose", "meshLoader", "geometry_types", "contacts"]
    if len(args) >= 2:
        if isinstance(args[1], str):
            arg_keys = [
                "root_joint",
                "root_joint_name",
                "verbose",
                "meshLoader",
                "geometry_types",
                "contacts",
            ]

    for key, arg in zip(arg_keys, args):
        if key in kwargs.keys():
            raise TypeError("Function got multiple values for argument ", key)
        else:
            kwargs[key] = arg

    return _buildModelsFromMJCF(filename, **kwargs)


def _buildModelsFromMJCF(
    filename,
    root_joint=None,
    root_joint_name=None,
    verbose=False,
    meshLoader=None,
    geometry_types=None,
    contacts=False,
):
    if geometry_types is None:
        geometry_types = [pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]

    contact_models = []
    if root_joint is None:
        model = pin.buildModelFromMJCF(filename)
    elif root_joint is not None and root_joint_name is None:
        model = pin.buildModelFromMJCF(filename, root_joint)
    else:
        model, contact_models = pin.buildModelFromMJCF(
            filename, root_joint, root_joint_name
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

    lst = [model]
    if contacts:
        lst.append(contact_models)

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
