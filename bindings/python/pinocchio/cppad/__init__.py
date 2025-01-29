#
# Copyright (c) 2020-2021 INRIA
#
# ruff: noqa: F401, F403, F405
# Manually register submodules
import sys

from .. import utils
from ..explog import exp, log
from ..pinocchio_pywrap_cppad import *
from ..pinocchio_pywrap_cppad import __raw_version__, __version__

sys.modules["pinocchio.cppad.rpy"] = rpy
sys.modules["pinocchio.cppad.cholesky"] = cholesky

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
