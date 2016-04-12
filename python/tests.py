#!/usr/bin/env python

import unittest, sys

from bindings import TestSE3  # noqa
from explog import TestExpLog  # noqa
from model import TestModel  # noqa
from rpy import TestRPY  # noqa
from utils import TestUtils  # noqa

if __name__ == '__main__':
    print "Python version"
    print sys.version_info
    unittest.main()
