3. Drag and Drop (aka Inverse kinematics)
=========================================

Objectives
----------

The main objective of the tutorial is to perform one or several tasks by
inverse kinematics, i.e. pseudo inversing a jacobian iteratively until
convergence of the task error.

Tutorial 3.0. Technical prerequisites
-------------------------------------

Robots ~~\~~~\~

We are going to use again the UR5 robot model, however this time mounted
as a mobile robot. The source code of the mobile robot is available
link:tp3\_mobilerobot\_py.html\[here\]. The robot has 3+6 DOF and can
move (2 translations + 1 rotation) freely on the plane. Two operation
frames have been defined: at the front of the basis, and at the tip of
the tool. They are displayed when the robot moves.

Example of how to use the robot is has below.

\[source,python\]
-----------------

import pinocchio as se3 from mobilerobot import MobileRobotWrapper from
pinocchio.utils import \*

pkg = '/home/student/models/' urdf = pkg +
'ur\_description/urdf/ur5\_gripper.urdf'

robot = MobileRobotWrapper(urdf,\[pkg,\])
robot.initDisplay(loadModel=True)

robot.viewer.gui.addFloor('world/floor')
========================================

NQ = robot.model.nq NV = robot.model.nv

q = rand(NQ) robot.display(q)

IDX\_TOOL = 24 IDX\_BASIS = 23

se3.framesKinematics(robot.model,robot.data) Mtool =
robot.data.oMf\[IDX\_TOOL\] Mbasis = robot.data.oMf\[IDX\_BASIS\] ----

Tutorial 3.1. Position the end effector
---------------------------------------

The first task will be concerned with the end effector. First define a
goal placement.

\[source,python\]
-----------------

from mobilerobot import M2v def place(name,M):
robot.viewer.gui.applyConfiguration(name,M2v(M))
robot.viewer.gui.refresh()

def Rquat(x,y,z,w): q = se3.Quaternion(x,y,z,w) q.normalize() return
q.matrix()

Mgoal = se3.SE3(Rquat(0.4,0.02, -0.5,0.7), np.matrix(\[.2,-.4,.7\]).T)
robot.viewer.gui.addXYZaxis('world/framegoal',\[1.,0.,0.,1.\],.015,4)
place('world/framegoal',Mgoal) ----

The current placement of the tool at configuration +q+ is available as
follows: \[source,python\] ---- IDX\_TOOL = 24
se3.forwardKinematics(robot.model,robot.data,q) \# Compute joint
placements se3.framesKinematics(robot.model,robot.data) \# Also compute
operational frame placements Mtool = robot.data.oMf\[IDX\_TOOL\] \# Get
placement from world frame o to frame f oMf ----

The desired velocity of the tool in tool frame is given by the log:
\[source,python\] ---- nu = se3.log(Mtool.inverse()\*Mgoal).vector ----

The tool Jacobian, also in tool frame, is available as follows:

\[source,python\]
-----------------

J = se3.frameJacobian(robot.model,robot.data,q,IDX\_TOOL,True,True)
-------------------------------------------------------------------

Pseudoinverse operator is available in +numpy.linalg+ toolbox.
\[source,python\] ---- from numpy.linalg import pinv ----

The integration of joint velocity vq in configuration q can be done
directly (q += vq dt). More generically, the se3 method integrate can be
used:

\[source,python\]
-----------------

q = se3.integrate(robot.model,q,vq\*dt) ----

*Question 1* Implement a for-loop that computes the jacobian and the
desired velocity in tool frame, and deduced the joint velocity using the
pseudoinverse. At each iteration, also integrate the current velocity
and display the robot configuration.

Tutorial 3.2. Position the basis on a line
------------------------------------------

A line displaying "x=0" is also displayed in Gepetto viewer. Next step
is to servo the front of the basis on this line.

Similarly, the distance of the basis frame to the line, with
corresponding jacobian, are: \[source,python\] ---- error =
Mbasis.translation\[0\] J =
se3.frameJacobian(robot.model,robot.data,q,IDX\_BASIS,False,True)\[0,:\]
----

Implement a second loop to servo the basis on the line. It becomes
interesting when both tasks are performed together. We can do that
simply by summing both tasks. For that, the numpy method +hstack+ can be
used to make a single error vector stacking the errors of tool and basis
tasks, and similarly for the jacobians. \[source,python\] ---- nu =
np.vstack(\[nu1, nu2\]) J = np.vstack(\[J1, J2\]) ----

However, it is stronger to move the basis only in the null space of the
basis. The null space projector of +J1+ can be computed using the
pseudoinverse. Following the control law performing task 1 and task 2 in
the null space of task 1 is:

\["latex"\] $vq_1 = J_1^+ v_1^*$ \["latex"\] $P_1 = I_9 - J_1^+ J_1$
\["latex"\] $vq_2 = vq_1 + (J_2 P_1)^+ ( v_2^* - J_2 vq_1)$

*Question 2* Implement two loops: the first one regulate the tool
placement alone. When the tool is properly placed, the second regulate
the tool placement and the basis position in the null-space of the tool.

//// Homework --------

Send by mail at nmansard@laas.fr a mail containing a single python file.
The subject of the mail should start with +\[SUPAERO\] TP3+ When
executed, the script should execute question 2. ////
