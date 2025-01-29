#
# Copyright (c) 2021 INRIA
#
# ruff: noqa: F401, F403, F405

# Manually register submodules
import sys

from .. import utils
from ..explog import exp, log
from ..pinocchio_pywrap_mpfr import *
from ..pinocchio_pywrap_mpfr import __raw_version__, __version__

sys.modules["pinocchio.mpfr.rpy"] = rpy
sys.modules["pinocchio.mpfr.cholesky"] = cholesky

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
