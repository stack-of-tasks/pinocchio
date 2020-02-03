#
# Copyright (c) 2015-2019 CNRS INRIA
#

import numpy
from .robot_wrapper import RobotWrapper
from .libpinocchio_pywrap import __version__, __raw_version__

from . import utils
from . import visualize
from .explog import exp, log
from .libpinocchio_pywrap import *
from .deprecated import *
from .shortcuts import *

# Manually register submodules
import sys
sys.modules['pinocchio.rpy'] = rpy
sys.modules['pinocchio.cholesky'] = cholesky

if pin.WITH_HPP_FCL:
  try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
  except ImportError:
    WITH_HPP_FCL_BINDINGS = False
else:
  WITH_HPP_FCL_BINDINGS = False
  
