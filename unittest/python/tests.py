#!/usr/bin/env python

from __future__ import print_function

import unittest, sys

from bindings import TestSE3  # TODO: probably remove and move/split its contents
from bindings_SE3 import TestSE3Bindings
from bindings_force import TestForceBindings
from bindings_motion import TestMotionBindings
from bindings_inertia import TestInertiaBindings
from bindings_frame import TestFrameBindings
from bindings_geometry_object import TestGeometryObjectBindings
from bindings_geometry_object_urdf import TestGeometryObjectUrdfBindings
from explog import TestExpLog
from model import TestModel
from rpy import TestRPY
from utils import TestUtils

if __name__ == '__main__':
    print("Python version")
    print(sys.version_info)
    unittest.main()
