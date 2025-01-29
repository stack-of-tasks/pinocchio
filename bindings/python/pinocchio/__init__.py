#
# Copyright (c) 2015-2021 CNRS INRIA
#
# ruff: noqa: E402, F401, F403, F405

import numpy

# On Windows, if pinocchio.dll is not in the same directory than
# the .pyd, it will not be loaded.
# We first try to load pinocchio, then, if it fail and we are on Windows:
#  1. We add all paths inside PINOCCHIO_WINDOWS_DLL_PATH to DllDirectory
#  2. If PINOCCHIO_WINDOWS_DLL_PATH we add the relative path from the
#     package directory to the bin directory to DllDirectory
# This solution is inspired from:
#  - https://github.com/PixarAnimationStudios/OpenUSD/pull/1511/files
#  - https://stackoverflow.com/questions/65334494/python-c-extension-packaging-dll-along-with-pyd
# More resources on https://github.com/diffpy/pyobjcryst/issues/33
try:
    from .pinocchio_pywrap_default import *
    from .pinocchio_pywrap_default import __raw_version__, __version__
except ImportError:
    import platform

    if platform.system() == "Windows":
        from .windows_dll_manager import build_directory_manager, get_dll_paths

        with build_directory_manager() as dll_dir_manager:
            for p in get_dll_paths():
                dll_dir_manager.add_dll_directory(p)
            from .pinocchio_pywrap_default import *
            from .pinocchio_pywrap_default import __raw_version__, __version__
    else:
        raise

import inspect

# Manually register submodules
import sys

from . import utils
from .explog import exp, log

submodules = inspect.getmembers(pinocchio_pywrap_default, inspect.ismodule)
for module_info in submodules:
    sys.modules["pinocchio." + module_info[0]] = module_info[1]

sys.modules["pinocchio.rpy"] = rpy
sys.modules["pinocchio.cholesky"] = cholesky

if WITH_COAL:
    try:
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

        WITH_COAL_BINDINGS = True
    except ImportError:
        WITH_COAL_BINDINGS = False
else:
    WITH_COAL_BINDINGS = False

from .deprecated import *
from .robot_wrapper import RobotWrapper
from .shortcuts import *
