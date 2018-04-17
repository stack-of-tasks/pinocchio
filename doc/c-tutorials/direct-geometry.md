# Direct Geometry

## Load the robot

```py
from pinocchio.robot_wrapper import RobotWrapper

PKG = '/opt/openrobots/share/'
URDF = PKG + 'ur5_description/urdf/ur5_gripper.urdf'

robot = RobotWrapper(URDF, [PKG])
```

## Visualize the model

launch `gepetto-gui`, and then

```py
robot.initDisplay(loadModel=True)
```

## Put the robot in a particular position

```py
import numpy as np
q = np.matrix([-.5, -1, 1.5, -.5, -.5, 0]).T
robot.display(q)
```
