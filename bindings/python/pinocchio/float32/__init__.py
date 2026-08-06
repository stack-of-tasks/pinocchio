#
# Copyright (c) 2026 Heriot-Watt University
#
"""Single-precision bindings for Pinocchio's kinematics and dynamics API.

Models, spatial types, algorithms, and NumPy results exposed here use ``float32``.
File parsers, parallel algorithms, and reachable-workspace helpers remain available
only from the main module. Geometry and collision objects keep their existing scalar
representation when used with a float32 model.
"""

# ruff: noqa: F401, F403, F405
# Manually register submodules
import inspect
import sys

from .. import pinocchio_pywrap_float32 as _pinocchio_pywrap_float32
from ..pinocchio_pywrap_float32 import *
from ..pinocchio_pywrap_float32 import __raw_version__, __version__
from . import utils
from .explog import exp, log

submodules = inspect.getmembers(_pinocchio_pywrap_float32, inspect.ismodule)
for module_info in submodules:
    sys.modules[__name__ + "." + module_info[0]] = module_info[1]

if WITH_COLLISION:
    import coal
    from coal import (
        CachedMeshLoader,
        CollisionGeometry,
        CollisionResult,
        Contact,
        DistanceResult,
        MeshLoader,
        StdVec_CollisionResult,
        StdVec_Contact,
        StdVec_DistanceResult,
    )

    # Pickling support becauso Vec3s is registered by
    # coal and pinocchio (see pinocchio/binding/python/multibody/data.hpp)
    coal.StdVec_Vec3s.__safe_for_unpickling__ = True
    coal.StdVec_Vec3s.__getstate_manages_dict__ = True

    # Deprecated, should be removed in next major release
    hppfcl = coal
