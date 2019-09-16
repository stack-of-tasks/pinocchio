# 1) Move your body (aka direct geometry)

## Objective

In this first series of exercises, we are going to start using the
library Pinocchio. We will load in the simulator the model of a simple
manipulator arm, the Universal Robot 5, or UR5. This model will be used
for a positioning task using simple methods. This set of exercices
emphasizes the first part of the class, about direct geometry.

The software Pinocchio is a C++ library provided with a python wrapping
that allows us to control it with a python terminal. Let's see how it
works.

## 1.0) Tips

### Setup

For this tutorial, you will need [Pinocchio](http://stack-of-tasks.github.io/pinocchio/download.html),
[Gepetto GUI](https://github.com/humanoid-path-planner/gepetto-viewer-corba), and the description of the ur5 robot.

For this, the easiest way is to add [robotpkg apt repository](http://robotpkg.openrobots.org/debian.html) and launch:
```
sudo apt install robotpkg-py27-pinocchio robotpkg-ur5-description robotpkg-py27-qt4-gepetto-viewer-corba robotpkg-osg-dae
```

### Python

We remind you that you can open a python terminal in
your own shell. Simply type :
```
student@student-virtualbox:~$ ipython
>>>
```
Afterwards you'll just have to type your commands in this newly opened terminal.

To close the python terminal, just type CTRL-D (CTRL-C first to
interrupt any on-going execution).

You can also write you command lines in a file and launch this script
without entering the interactive mode (ie. without starting a new python
terminal). In your shell, just type:
```
student@student-virtualbox:~$ ipython script.py
>>>
```

### Pinocchio

#### Basic mathematical objects:

In the following, we will use numpy `Matrix` class to represent matrices
and vectors. In numpy, vectors simply are matrices with one column. See
the following example.

```py
import numpy as np
A = np.matrix([[1, 2, 3, 4], [5, 6, 7, 8]])  # Define a 2x4 matrix
b = np.zeros([4, 1])  # Define a 4 vector (ie a 4x1 matrix) initialized with 0
c = A * b             # Obtain c by multiplying A by b.
```

A bunch of useful functions are packaged in the utils of pinocchio.

```py
from pinocchio.utils import *
eye(6)                      # Return a 6x6 identity matrix
zero(6)                     # Return a zero 6x1 vector
zero([6, 4])                # Return az zero 6x4 matrix
rand(6)                     # Random 6x1 vector
isapprox(zero(6), rand(6))  # Test epsilon equality
mprint(rand([6, 6]))        # Matlab-style print
skew(rand(3))               # Skew "cross-product" 3x3 matrix from a 3x1 vector
cross(rand(3), rand(3))     # Cross product of R^3
rotate('x', 0.4)            # Build a rotation matrix of 0.4rad around X.
```

Specific classes are defined to represent objects of \f$SE(3)\f$, \f$se(3)\f$ and
\f$se(3)^*\f$. Rigid displacements, elements of \f$SE(3)\f$, are represented by
the class `SE3`.

```py
import pinocchio as pin
R = eye(3); p = zero(3)
M0 = pin.SE3(R, p)
M = pin.SE3.Random()
M.translation = p; M.rotation = R
```

Spatial velocities, elements of \f$se(3) = M^6\f$, are represented by the
class `Motion`.

```py
v = zero(3); w = zero(3)
nu0 = pin.Motion(v, w)
nu = pin.Motion.Random()
nu.linear = v; nu.angular = w
```

Spatial forces, elements of \f$se(3)^* = F^6\f$, are represented by the
class `Force`.

```py
f = zero(3); tau = zero(3)
phi0 = pin.Force(f, tau)
phi = pin.Force.Random()
phi.linear = f; phi.angular = tau
```

## 1.1) Creating and displaying the robot

### Robot kinematic tree

The kinematic tree is represented by two C++ objects called Model (which
contains the model constants: lengths, masses, names, etc) and Data
(which contains the working memory used by the model algorithms). Both
C++ objects are contained in a unique Python class. The first class is
called RobotWrapper and is generic.

For the next steps, we are going to work with the RobotWrapper.

Import the class `RobotWrapper` and create an instance of this class in
the python terminal. At initialization, RobotWrapper will read the model
description in the URDF file given as argument. In the following, we
will use the model of the UR5 robot, available in the directory "models"
of pinocchio (available in the homedir of the VBox).

```py
from pinocchio.robot_wrapper import RobotWrapper

URDF = '/opt/openrobots/share/ur5_description/urdf/ur5_gripper.urdf'
robot = RobotWrapper.BuildFromURDF(URDF)
```

The code of the RobotWrapper class is in
`/opt/openrobots/lib/python2.7/site-packages/pinocchio/robot_wrapper.py`.
Do not hesitate to have a look at it and to take inspiration from the
implementation of the class functions.

UR5 is a fixed robot with one 6-DOF arms developed by the Danish company
Universal Robot. All its 6 joints are revolute joints. Its configuration
is in R^6 and is not subject to any constraint. The model of UR5 is
described in a URDF file, with the visuals of the bodies of the robot
being described as meshed (i.e. polygon soups) using the Collada format
".dae". Both the URDF and the DAE files are available in the package
`robotpkg-ur5-description`

### Exploring the model

The robot model is available in `robot.model`. It contains the names of
all the robot joint `names`, the kinematic tree `parents` (i.e. the
graph of parents, 0 being the root and having no parents), the position
of the current joint in the parent coordinate frame `jointPosition`,
the mass, inertia and center-of-gravity position of all the bodies
(condensed in a spatial inertia 6x6 matrix) `inertias` and the gravity
of the associated world `gravity`. All these functions are documented
and are available in the correponding class dictionnary.

```py
for name, function in robot.model.__class__.__dict__.items():
    print(' **** %s: %s' % (name, function.__doc__))
```

Similarly, the robot data are available in `robot.data`. All the variables
allocated by the classical rigid-body dynamics algorithms are stored in
`robot.data` and are available through the python wrapping. Similarly to
the model object, the function are documented and are available from the
class dictionnary. The most useful in the following will be the
placement of the frame associated which each joint output stored in
`robot.data.oMi`.

For example, the robot end effector corresponds to the output of the
last joint, called `wrist_1_joint`. The ID of the joint in the joint
list can be recovered from its name, and then used to access its
placement:

```py
# Get index of end effector

idx = robot.index('wrist_3_joint')

# Compute and get the placement of joint number idx

placement = robot.placement(q, idx)
# Be carreful, Python always returns references to values.
# You can often .copy() the object to avoid side effects
# Only get the placement
placement = robot.data.oMi[idx].copy()
```

Finally, some recurring datas (used in Model and Data) have been wrapped
to functions in some python shortcuts, also available in RomeoWrapper:

- The size of the robot configuration is given by `nq`.
- The dimension of its tangent space (velocity) is `nv`.
- The index of a joint in the tree can be accessed from its name by index (see above).
- The classical algorithms are also binded: com, Jcom, mass, biais, joint gravity, position and velocity of each joint.

```py
q = zero(robot.nq)
v = rand(robot.nv)
robot.com(q)  # Compute the robot center of mass.
robot.placement(q, 3)  # Compute the placement of joint 3
```

### Display the robot

To display the robot, we need an external program called *Gepetto
Viewer*. If you completed the installation in the previous page, you can
launch this program, open a new terminal in an empty workspace.

```
student@student-virtualbox:~$ gepetto-gui
```

This will start a server waiting for instructions. We will now create a client that will
ask the server to perform some requests (such as creating a window or
displaying our robot)

In a python terminal you can now load the visual model of the robot in
the viewer:

```py
robot.initViewer(loadModel=True)
```

This will flush the robot model inside the GUI. The argument
`loadModel=True` is mandatory when you start or restart the GUI. In
later call to your scripts, you can set the argument to `False`. A side
effector of `=True` is that it will move the viewpoint inside the GUI to
a reference zero position.

### More details about loading the model (optionnal)

You can access the visual object composing the robot model by
`robot.visual_model.geometryObject`.

```py
visualObj = robot.visual_model.geometryObjects[4]  # 3D object representing the robot forarm
visualName = visualObj.name                        # Name associated to this object
visualRef = robot.getViewerNodeName(visualObj, pin.GeometryType.VISUAL)    # Viewer reference (string) representing this object
```

Moving one object

```py
q1 = (1, 1, 1, 1, 0, 0, 0)  # x, y, z, quaternion
robot.viewer.gui.applyConfiguration(visualRef, q1)
robot.viewer.gui.refresh()  # Refresh the window.
```

Additional objects can be created, like a sphere as follows.

```py
rgbt = [1.0, 0.2, 0.2, 1.0]  # red, green, blue, transparency
robot.viewer.gui.addSphere("world/sphere", .1, rgbt)  # .1 is the radius
```

The exhaustive list of the object that can be created is available in
the IDL of the GUI:
`/opt/openrobots/share/idl/gepetto/corbaserver/graphical-interface.idl`

## 1.2) Simple pick and place

*Objectives:* Display the robot at a given configuration or along a
given trajectory

### Pick:

Say we have a target at position `[.5, .1, .2]` and we would like the
robot to grasp it.

```py
robot.viewer.gui.applyConfiguration("world/sphere", (.5, .1, .2, 1.,0.,0.,0. ))
robot.viewer.gui.refresh()  # Refresh the window.
```

First display a small sphere at this position to visualize it.

Then decide by any mean you want a configuration of the robot so that
the end effector is touching the sphere.

At the reference position you built, the end effector placement can be
obtained by `robot.placement(q, 6)`. Only the translation part of the
placement has been selected. The rotation is free.

*Optional* Say now that the object is a rectangle and not a sphere.
Pick the object at a reference position with the rotation that is
imposed, so that the end effector is aligned with one of the faces of
the rectangle.

### Place:

Choose any trajectory you want in the configuration space, starting from
the reference position built in the previous exercice (it can be
sinus-cosinus waves, polynomials, splines, straight lines).

Make a for loop to display the robot at sampling positions along this
trajectory. The function sleep in module time (from time import sleep)
can be used to slow down the loop.

At each instant of your loop, recompute the position of the ball and
display it so that it always "sticks" to the robot end effector.
