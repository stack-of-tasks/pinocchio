#
# Copyright (c) 2022 INRIA
#
from ..pinocchio_pywrap_cppadcg import *

from ..pinocchio_pywrap_cppadcg import __version__, __raw_version__

from .. import utils
from ..explog import exp, log

# Manually register submodules
import sys

sys.modules["pinocchio.cppadcg.rpy"] = rpy
sys.modules["pinocchio.cppadcg.cholesky"] = cholesky

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
