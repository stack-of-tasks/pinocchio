#
# Copyright (c) 2022 INRIA
#
# ruff: noqa: F401, F403, F405
# Manually register submodules
import sys

from .. import utils
from ..explog import exp, log
from ..pinocchio_pywrap_cppadcg import *
from ..pinocchio_pywrap_cppadcg import __raw_version__, __version__

sys.modules["pinocchio.cppadcg.rpy"] = rpy
sys.modules["pinocchio.cppadcg.cholesky"] = cholesky

if WITH_HPP_FCL:
    try:
        import hppfcl
        from hppfcl import (
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

        WITH_HPP_FCL_BINDINGS = True
    except ImportError:
        WITH_HPP_FCL_BINDINGS = False
else:
    WITH_HPP_FCL_BINDINGS = False
