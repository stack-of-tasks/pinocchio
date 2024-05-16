#
# Copyright (c) 2020-2021 INRIA
#
from ..pinocchio_pywrap_cppad import *

from ..pinocchio_pywrap_cppad import __version__, __raw_version__

from .. import utils
from ..explog import exp, log

# Manually register submodules
import sys

sys.modules["pinocchio.cppad.rpy"] = rpy
sys.modules["pinocchio.cppad.cholesky"] = cholesky

if WITH_HPP_FCL:
    try:
        import hppfcl
        from hppfcl import (
            Contact,
            StdVec_Contact,
            CollisionResult,
            StdVec_CollisionResult,
            DistanceResult,
            StdVec_DistanceResult,
            CollisionGeometry,
            MeshLoader,
            CachedMeshLoader,
        )

        WITH_HPP_FCL_BINDINGS = True
    except ImportError:
        WITH_HPP_FCL_BINDINGS = False
else:
    WITH_HPP_FCL_BINDINGS = False
