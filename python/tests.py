#!/usr/bin/env python

import unittest

from bindings import TestSE3  # noqa
from explog import TestExpLog  # noqa
from model import TestModel  # noqa
from rpy import TestRPY  # noqa
from utils import TestUtils  # noqa

if __name__ == '__main__':
    unittest.main()
