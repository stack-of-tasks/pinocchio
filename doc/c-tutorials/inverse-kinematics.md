# Inverse Kinematics

## Get the robot

```py
import pinocchio
from os.path import join

PKG = '/opt/openrobots/share'
URDF = join(PKG, 'ur5_description/urdf/ur5_gripper.urdf')

robot = pinocchio.robot_wrapper.RobotWrapper(URDF, [PKG])
robot.initDisplay(loadModel=True)

#robot.viewer.gui.addFloor('world/floor')

NQ = robot.model.nq

q = pinocchio.utils.rand(NQ)
robot.display(q)
```

## Get current and desired position of end effector

```py
import numpy as np

IDX_TOOL = 22
Mtool = robot.data.oMf[IDX_TOOL]

Tgoal = [.2, -.4, .7]
Qgoal = [.4, .02, -.5, .7]

Qgoal = pinocchio.Quaternion(*Qgoal)
Qgoal.normalize()
Mgoal = pinocchio.SE3(Qgoal.matrix(), np.matrix(Tgoal).T)
```

## Show goal

```py
robot.viewer.gui.addXYZaxis('world/goal', [1, 0, 0, 1], .015, .2)
Vgoal = pinocchio.utils.XYZQUATToViewerConfiguration(pinocchio.utils.se3ToXYZQUAT(Mgoal))
robot.viewer.gui.applyConfiguration('world/goal', Vgoal)
robot.viewer.gui.refresh()
```

## Reach it

```py
import time
from numpy.linalg import norm, pinv

DT = .001
nu = pinocchio.utils.zero(6) + 1

while norm(nu) > .01:
    pinocchio.forwardKinematics(robot.model, robot.data, q)
    pinocchio.framesKinematics(robot.model, robot.data)
    Mtool = robot.data.oMf[IDX_TOOL]
    nu = pinocchio.log(Mtool.inverse() * Mgoal).vector
    J = pinocchio.frameJacobian(robot.model, robot.data, IDX_TOOL, q)
    vq = pinv(J) * nu
    robot.increment(q, vq * DT)
    robot.display(q)
    time.sleep(DT)
```
