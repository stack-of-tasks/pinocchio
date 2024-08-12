#
# Copyright (c) 2016 CNRS
#

import numpy as np
import pinocchio as pin
from numpy.linalg import norm
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import rand

from dcrba import DCRBA, DRNEA, Coriolis

np.random.seed(0)

robot = RobotWrapper(
    "/home/nmansard/src/pinocchio/pinocchio/models/romeo/urdf/romeo.urdf",
    [
        "/home/nmansard/src/pinocchio/pinocchio/models/romeo/",
    ],
    pin.JointModelFreeFlyer(),
)
q = rand(robot.model.nq)
q[3:7] /= norm(q[3:7])
vq = rand(robot.model.nv)
aq = rand(robot.model.nv)

# d/dq M(q)
dcrba = DCRBA(robot)
dcrba.pre(q)
Mp = dcrba()

# d/dvq RNEA(q,vq) = C(q,vq)
coriolis = Coriolis(robot)
C = coriolis(q, vq)

# d/dq RNEA(q,vq,aq)
drnea = DRNEA(robot)
aq = rand(robot.model.nv)
R = drnea(q, vq, aq)
