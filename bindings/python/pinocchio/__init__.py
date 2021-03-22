#
# Copyright (c) 2015-2020 CNRS INRIA
#

import numpy
from .pinocchio_pywrap import *

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
