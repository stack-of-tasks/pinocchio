#
# Copyright (c) 2016 CNRS
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

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *
from dcrba import *

np.random.seed(0)

robot = RobotWrapper('/home/nmansard/src/pinocchio/pinocchio/models/romeo/urdf/romeo.urdf',
                     [ '/home/nmansard/src/pinocchio/pinocchio/models/romeo/', ],
                     se3.JointModelFreeFlyer()
                     )
q  = rand(robot.model.nq); q[3:7] /= norm(q[3:7])
vq = rand(robot.model.nv)
aq = rand(robot.model.nv)

# d/dq M(q)
dcrba = DCRBA(robot)
dcrba.pre(q)
Mp = dcrba()

# d/dvq RNEA(q,vq) = C(q,vq)
coriolis = Coriolis(robot)
C = coriolis(q,vq)

# d/dq RNEA(q,vq,aq)
drnea = DRNEA(robot)
aq    = rand(robot.model.nv)
R = drnea(q,vq,aq)


