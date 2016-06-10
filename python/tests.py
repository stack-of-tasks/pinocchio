#!/usr/bin/env python

import unittest, sys

from bindings import TestSE3  # noqa
from bindings_SE3 import TestSE3Bindings
from bindings_force import TestForceBindings
from bindings_motion import TestMotionBindings
from bindings_inertia import TestInertiaBindings
from bindings_frame import TestFrameBindings
from bindings_geometry_object import TestGeometryObjectBindings  # Python Class RobotWrapper needs geometry module to not raise Exception
from explog import TestExpLog  # noqa
from model import TestModel  # noqa
from rpy import TestRPY  # noqa
from utils import TestUtils  # noqa

if __name__ == '__main__':
    print "Python version"
    print sys.version_info
    unittest.main()
