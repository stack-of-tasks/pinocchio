#
# Copyright (c) 2015-2020 CNRS INRIA
#

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
    from .pinocchio_pywrap import *
except ImportError:
    import os
    import sys
    import platform
    if platform.system() == "Windows":
        __pinocchio_paths = os.getenv('PINOCCHIO_WINDOWS_DLL_PATH')
        if __pinocchio_paths is None:
            # Standard site-packages to bin path
            RELATIVE_DLL_PATH = "..\\..\\..\\bin"
            __dll_paths = [os.path.join(os.path.dirname(__file__), RELATIVE_DLL_PATH)]
        else:
            __dll_paths = __pinocchio_paths.split(os.pathsep)
        if sys.version_info >= (3, 8):
            for p in __dll_paths:
                # add_dll_directory can fail on relative path and non
                # existing path.
                # Since we don't know all the fail criterion we just ignore
                # thrown exception
                try:
                    os.add_dll_directory(p)
                except OSError:
                    pass
        else:
            for p in __dll_paths:
                os.environ["PATH"] += os.pathsep + p
        from .pinocchio_pywrap import *
    else:
        raise

from .pinocchio_pywrap import __version__, __raw_version__

from . import utils
from .explog import exp, log

# Manually register submodules
import sys, inspect

submodules = inspect.getmembers(pinocchio_pywrap, inspect.ismodule)
for module_info in submodules:
  sys.modules['pinocchio.' + module_info[0]] = module_info[1]

if WITH_HPP_FCL:
  try:
    import hppfcl
    from hppfcl import Contact, StdVec_Contact, CollisionResult, StdVec_CollisionResult, DistanceResult, StdVec_DistanceResult, CollisionGeometry, MeshLoader, CachedMeshLoader
    WITH_HPP_FCL_BINDINGS = True
  except ImportError:
    WITH_HPP_FCL_BINDINGS = False
else:
  WITH_HPP_FCL_BINDINGS = False

from .robot_wrapper import RobotWrapper
from .deprecated import *
from .shortcuts import *
from . import visualize
