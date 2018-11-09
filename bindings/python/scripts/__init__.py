#
# Copyright (c) 2015-2016,2018 CNRS
#

import numpy as np
from pinocchio.robot_wrapper import RobotWrapper

from . import libpinocchio_pywrap as se3
from . import utils
from .explog import exp, log
from .libpinocchio_pywrap import *
from .deprecated import *
from .shortcuts import *

se3.AngleAxis.__repr__ = lambda s: 'AngleAxis(%s)' % s.vector()

