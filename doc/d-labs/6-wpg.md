6. Take a walk (aka optimal control)
====================================

Objective
---------

The objective of this work is to get a first touch of optimal control
The tutorial will guide you to generate a dynamically balanced walk
motion for a humanoid robot using a LQR to compute the robot
center-of-mass trajectory.

Tutorial 6.0: prerequesites
---------------------------

*Prerequesite \#1* A humanoid robot models, with at least two legs.

*Prerequesite \#2* An inverse geometry solver based on BFGS.

Yet, the inverse geometry only solves the motion of the robot for a
constant task, like reaching a specific position of the hand, or a
constant position of the center of mass.

It is possible to modify the BFGS call to perform an inverse kinematics
by (i) limiting the number of iteration of BFGS to a small value e.g 10
iterations maximum, (ii) initializing the non-linear search from the
previous configuration of the robot, and (iii) turning off the default
verbose output of BFGS. For example, the robot can track a target moving
vertically using the following example:

\[source,python\]
-----------------

cost.Mdes = se3.SE3( eye(3),np.matrix(\[0.2,0,0.1+t/100.\]) ) \#
Reference target at time 0. q = np.copy(robot.q0) for t in range(100):
cost.Mdes.translation = np.matrix(\[0.2,0,0.1+t/100.\]) q =
fmin\_bfgs(cost, q,maxiter=10,disp=False) robot.display(q) ----

Implement a motion of the right foot of the robot tracking a straight
line from the initial position of the robot to a position 10cm forward,
while keeping a constant rotation of the foot.

Tutorial 6.1: defining input
----------------------------

The input of the walk generation algorithm is a sequence of steps with a
given timing. This input will be represented by two sequences as in the
examples below. The class +FootSteps+ (link:footsteps.html\[*see code
link*\]) can be used to define, store and access to the footstep plan.

\[source, python\]
------------------

Define 6 steps forward, starting with the left foot and stoping at the same forward position.
=============================================================================================

footsteps = FootSteps( \[0.0,-0.1\] , \[0.0,0.1\] ) footsteps.addPhase(
.3, 'none' ) footsteps.addPhase( .7, 'left' , \[0.1,+0.1\] )
footsteps.addPhase( .1, 'none' ) footsteps.addPhase( .7, 'right',
\[0.2,-0.1\] ) footsteps.addPhase( .1, 'none' ) footsteps.addPhase( .7,
'left' , \[0.3,+0.1\] ) footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', \[0.4,-0.1\] ) footsteps.addPhase( .1,
'none' ) footsteps.addPhase( .7, 'left' , \[0.5,+0.1\] )
footsteps.addPhase( .1, 'none' ) footsteps.addPhase( .7, 'right',
\[0.5,-0.1\] ) footsteps.addPhase( .5, 'none' ) ----

The class stores three functions of time: left, right and flyingFoot.
Each function is piecewise constant. For each function, the user can ask
what is the value of this function at time t.

The storage is composed of three lists for left, right and flyingFoot,
and a list for time. The list of times stores the time intervals, i.e.
each element of the list is the start of a time interval. The first
element of the list is 0. The value of the functions
left,right,flyingFoot one this time interval is stored at the same
position is their respective list (i.e. value of left on interval \[
time\[i\],time\[i+1\] \] is stored in left\[i\].

The 4 lists are set up using function addPhase(). The values of
functions left,right,flyingFoot can be accessed through the function
getPhaseType(t), getLeftPosition(t), getRightPosition(t). PhaseType are
'left' (meaning left foot is flying, right foot is fixed), 'right' (ie
the opposite) or 'none' (meaning no foot is flying, both are fixed on
the ground).

Additionnally, functions getLeftNextPosition(t), getRightNextPosition(t)
can be used to get the next position of the flying foot (in that case,
additional work is needed to compute the position of flying foot at time
t by interpolating getLeftPosition(t) and getLeftNextPosition(t).

Functions getPhaseStart(t), getPhaseDuration(t) and getPhaseRemaining(t)
can be used to get the starting time, the duration and the remaining
time of the current phase at time t.

A phase 'none' defines a double support phase (no foot moving). A phase
'left' (resp. 'right') defines a simple support phase while indicating
the flying foot. The time is a duration. The position is absolute.

Each interval corresponds to the following constant support phases:

\["latex"\]
\begin{tabular}{lll}
$\left[t_0,t_1\right]$ &double support phase& left foot in steps [0], right foot in steps [1] \\
$\left[t_1,t_2\right]$ &left foot support phase& right foot moving to steps [2]\\
$\left[t_2,t_3\right]$ &double support phase,& \\
$\left[t_3,t_4\right]$ &right foot support phase,& left foot moving to steps [3]\\
&\vdots&\\
$\left[t_{m-2}, t_{m-1}\right]$ &double support phase,& left foot in steps [p-2], right foot in steps [p-1]
\end{tabular}
\[source,python\]
-----------------

Example of use
==============

footsteps.getPhaseType(.4) \# return 'left'
footsteps.getLeftPosition(0.4) \# return 0,0.1
footsteps.getLeftNextPosition(0.4) \# return 0.1,0.1
footsteps.getPhaseStart(0.4) \# return 0.3
footsteps.getPhaseDuration(0.4) \# return 0.7
footsteps.getPhaseRemaining(0.4) \# return 0.6
footsteps.isDoubleFromLeftToRight(0) \# return False
footsteps.isDoubleFromLeftToRight(1) \# return True ----

Tutorial 6.2: computing reference ZMP
-------------------------------------

Implement a python class called +ZmpRef+ that takes as input a sequence
of times and a sequence of steps. Objects of this class behave as a
function of time that returns a 2 dimensional vector:

\[source, python\]
------------------

> > > zmp = ZmpRef (footsteps) zmp (2.5) array(\[ 0.41 , 0.096\]) ----

The function should be a piecewise affine function

-   starting in the middle of the ankles of the two first steps,
-   finishing in the middle of the two ankles of the two last steps,
-   constant under the foot support during single support phases.

You can use the template below.

\[source, python\]
------------------

class ZmpRef (object): def **init** (self, footsteps) : self.footsteps =
footsteps \# Operator () def **call** (self, t): return array
(self.steps \[0\]) ----

For the inputs provided above, the graph of +zmp+ is given below.

image::images/zmp-ref.png\[width="100%",alt="zmp\_ref against time"\]

Tutorial 6.3: reference trajectory of the center of mass
--------------------------------------------------------

Using the reference zmp trajectory implemented in Tutorial 6.3,
implement a class +ComRef+ that computes the reference trajectory of the
center of mass by optimal control.

To write the underlying optimization problem, you can use a factor
graph. A simple implementation is available in
link:factor\_graph.html\[(*see the source code*)\]. An example of use is
the following. Try to guess the solution before executing it.

\[source,python\]
-----------------

f = FactorGraph(1,5) \# Define a factor of 5 variables of dimension 1

M = eye(1) \# M is simply 1 written as a 1x1 matrix. for i in range(4):
f.addFactorConstraint( \[ Factor( i,M ), Factor( i+1,-M ) \], zero(1) )

f.addFactor( \[ Factor(0,M) \], M*10 ) f.addFactor( \[ Factor(4,M) \],
M*20 )

x = f.solve()
-------------

Tutorial 6.4: reference trajectories of the feet
------------------------------------------------

Using the same method as in Tutorial 6.2, implement two classes
+RightAnkleRef+ and +LeftAnkleRef+ that return reference positions of
the ankles as homogeneous matrices. Unlike zmp reference, trajectories
of the feet should be continuously differentiable.

Tutorial 6.5: generate walk motion
----------------------------------

Use the classes defined in the previous sections to generate a walk
motion using the inverse kinematics solver of Tutorial 2.
