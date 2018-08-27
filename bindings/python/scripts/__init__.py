#
# Copyright (c) 2015-2016,2018 CNRS
#
# This file is part of Pinocchio
# Pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# Pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# Pinocchio If not, see
# <http://www.gnu.org/licenses/>.

import numpy as np
from pinocchio.robot_wrapper import RobotWrapper

from . import libpinocchio_pywrap as se3
from . import utils
from .explog import exp, log
from .libpinocchio_pywrap import *
from .deprecated import *

se3.AngleAxis.__repr__ = lambda s: 'AngleAxis(%s)' % s.vector()

