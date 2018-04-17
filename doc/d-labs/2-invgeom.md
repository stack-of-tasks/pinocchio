2. Grasp an object (aka inverse Geometry)
=========================================

Objectives
----------

The main objective of the first tutorial is to compute a configuration
of the robot minimizing a cost (maximizing a reward) and respecting some
given constraints. Additionally, the objective is to have a first hands
on the difficulties of working outside of real vector spaces and to
consider what are the guesses that are taken by an optimal solver.

Tutorial 2.0. Technical prerequisites
-------------------------------------

Python SciPy and MatplotLib ~~~~~~~~~~~\~~~~~~~~~~~~\~~\~~

You will need the two libraries Python +SciPy+ (scientific Python) and
+MatPlotLib+ (plot mathematical data).

SciPy can be installed by +sudo apt-get install python-scipy+. It is
already installed if you are using the VirtualBox. Examples of calls of
these two functions are given below. We will use both solvers with
numerical (finite-differencing) differenciation, to avoid the extra work
of differencing the cost and constraint functions by hand. In general,
it is strongly advice to first test a numerical program with finite
differencing, before implementing the true derivatives only if needed.
In any case, the true derivatives must always be checked by comparing
the results with the finite differenciation.

Additionally, the provided implementation of BFGS allows the user to
provide a callback function and track the path taken by the solver, but
does not provide the possibility to specify constraints (constraints can
be added as penalty functions in the cost, but this requires additional
work). The constrained least-square implementation allows the user to
specify equality and inequality constraints, but not the callback. In
the following, start to use BFGS before moving to the constrained
least-square only when constraints are really needed. \[source,python\]
---- \# Example of use a the optimization toolbox of SciPy. import numpy
as np from scipy.optimize import fmin\_bfgs, fmin\_slsqp

def cost(x): '''Cost f(x,y) = x\^2 + 2y\^2 - 2xy - 2x ''' x0 = x\[0\] x1
= x\[1\] return -1*(2*x0*x1 + 2*x0 - x0\*\*2 - 2\*x1\*\*2)

def constraint\_eq(x): ''' Constraint x\^3 = y ''' return np.array(\[
x\[0\]\*\*3-x\[1\] \])

def constraint\_ineq(x): '''Constraint x&gt;=2, y&gt;=2''' return
np.array(\[ x\[0\]-2,x\[1\]-2 \])

class CallbackLogger: def **init**(self): self.nfeval = 1 def
**call**(self,x): print '===CBK=== {0:4d} {1: 3.6f} {2: 3.6f} {3:
3.6f}'.format(self.nfeval, x\[0\], x\[1\], cost(x)) self.nfeval += 1

x0 = np.array(\[0.0,0.0\]) \# Optimize cost without any constraints in
BFGS, with traces. xopt\_bfgs = fmin\_bfgs(cost, x0,
callback=CallbackLogger()) print '\n *\*\* Xopt in BFGS =
',xopt\_bfgs,'\n\n\n\n'

Optimize cost without any constraints in CLSQ
=============================================

xopt\_lsq = fmin\_slsqp(cost,\[-1.0,1.0\], iprint=2, full\_output=1)
print '\n *\*\* Xopt in LSQ = ',xopt\_lsq,'\n\n\n\n'

Optimize cost with equality and inequality constraints in CLSQ
==============================================================

xopt\_clsq = fmin\_slsqp(cost,\[-1.0,1.0\], f\_eqcons=constraint\_eq,
f\_ieqcons=constraint\_ineq, iprint=2, full\_output=1) print '\n *\*\*
Xopt in c-lsq = ',xopt\_clsq,'\n\n\n\n' ---- Take care that all +SciPy+
always works with vectors represented as 1-dimensional array, while
Pinocchio works with vectors represented as matrices (which are in fact
two-dimensional arrays, with the second dimension being 1). You can pass
from a SciPy-like vector to a Pinocchio-like vector using:
\[source,python\] ---- import numpy as np x = np.array(\[ 1.0, 2.0, 3.0
\]) q = np.matrix(x).T x = q.getA()\[:,0\] ----

The second library +MatPlotLib+ plots values on a 2D graph. Once more,
documentation is available here:
https://www.labri.fr/perso/nrougier/teaching/matplotlib/ An example is
provided below.

\[source, python\]
------------------

import numpy as np import matplotlib.pyplot as plt \# In plt, the
following functions are the most useful: \#
ion,plot,draw,show,subplot,figure,title,savefig

For use in interactive python mode (ipthyon -i)
===============================================

interactivePlot = False

if interactivePlot: plt.ion() \# Plot functions now instantaneously
display, shell is not blocked

Build numpy array for x axis
============================

x = 1e-3 \* np.array (range (100)) \# Build numpy array for y axis y =
x\*\*2

fig = plt.figure () ax = fig.add\_subplot ('111') ax.plot (x, y)
ax.legend (("x\^2",))

if not interactivePlot: \# Display all the plots and block the shell. \#
The script will only ends when all windows are closed. plt.show () ----

Robots ~~\~~~\~

We mostly use here the model UR5, used in the first tutorial. Refer to
the instructions of Tuto 1 to load it.

Optionally, we might want to use a more complex robot model. Romeo is a
humanoid robot developed by the French-Japanese company Aldebaran
Robotics. It has two legs, two arms and a head, for a total of 31 joints
(plus 6DOF on the free flyer). Its model is natively in Pinocchio (no
additional dowload needed). Romeo can be loaded with: \[source,python\]
---- import pinocchio as se3 from pinocchio.romeo\_wrapper import
RomeoWrapper path =
'/home/nmansard/src/pinocchio/pinocchio/models/romeo/' urdf = path +
'urdf/romeo.urdf' \# Explicitly specify that the first joint is a free
flyer. robot = RomeoWrapper(urdf,\[path,\]) \# Load urdf model
robot.initDisplay(loadModel=True) ---- Additionally, the index of right
and left hands and feet are stored in romeo.rh, romeo.lh, romeo.rf and
romeo.lf.

Tutorial 2.1. Position the end effector
---------------------------------------

The first tutorial is to position (i.e. translation only) the end
effector of a manipulator robot to a given position. For this first
part, we will use the fixed serial-chain robot model.

Recall first that the position (3D) of the joint with index "i" at
position "q" can be access by the following two lines of code:
\[source,python\] ---- \# Compute all joint placements and put the
position of joint "i" in variable "p". import pinocchio as se3
se3.forwardKinematics(robot.model,robot.data,q) p =
robot.data.oMi\[i\].translation ----

*Question 1* Using this, build a cost function to be the norm of the
difference between the end-effector position +p+ and a desired position
+pdes+. The cost function is a function that accepts as input an
1-dimensional array and return a float.

*Question 2* Then use +fmin\_bfgs+ to find a configuration q with the
end effector at position +pdes+.

*Question 3* Finally, implements a callback function that display in
Gepetto-Viewer every candidate configuration tried by the solver.

Tutorial 2.2. Approaching the redundancy (optionnal)
----------------------------------------------------

The manipulator arm has 6 DOF, while the cost function only constraints
3 of them (the position of the end effector). A continuum of solutions
then exists. The two next questions are aiming at giving an intuition of
this continuum.

*Question 4* Sample several configurations respecting +pdes+ by giving
various initial guesses to the solver. Store this sampling of solutions
in a list, then display this list in Gepetto-Viewer, each configuration
begin displayed during 1 second (pause of 1 seconds can be obtained
using: import time; time.sleep(1.0)).

A configurations in this continuum can then be selected with particular
properties, like for example being the closest to a reference
configuration, or using some joints more than the others, or any other
criterion that you can imagine.

*Question 5* Sum a secondary cost term to the first positioning cost, to
select the posture that maximizes the similarity (minimizes the norm of
the difference) to a reference posture. The relative importance of the
two cost terms can be adjusted by weighting the sum: find the weight so
that the reference position is obtained with a negligible error (below
millimeter) while the posture is properly taken into account.

Tutorial 2.3. Placing the end-effector
--------------------------------------

The next step is to find a configuration of the robot so that the end
effector respects a reference placement, i.e. position and orientation.
The stake is to find a metric in SE(3) to continuously quantify the
distance between two placements. There is no canonical metric in SE(3),
i.e. no absolute way of weighting the position with respect to the
orientation. Two metrics can be considered, namely the log in SE(3) or
in R\^3 x SE(3). The tutorial will guide you through the first choice.

The SE(3) and SO(3) logarithm are implemented in Pinocchio in the explog
module. \[source,python\] ---- from pinocchio.explog import log from
pinocchio import SE3 nu = log(SE3.Random()) nu\_vec = nu.vector ----

*Question 6* Solve for the configuration that minimizes the norm of the
logarithm of the difference between the end effector placement and the
desired placement.

Optionally, try other metrics, like the log metric of R\^3 x SO(3), or
the Froebenius norm of the homogeneous matrix.

Tutorial 2.4. Working with a mobile robot (optionnal)
-----------------------------------------------------

Until now, the tutorial only worked with a simple manipulator robot,
i.e. whose configuration space is a real vector space. Consider now the
humanoid robot, whose first joint is a free joint: it has 6 degrees of
freedom (3 rotations, 3 translations) but its configuration vector is
dimension 7. You can check it with +robot.model.nq+, that stores the
dimension of the configuration, and +robot.model.nv+, that stores the
dimension of the configuration velocity, i.e. the number of degrees of
freedom. For the humanoid, nq = nv+1.

Indeed, the configuration coefficients 3 to 7 are indeed representing a
quaternion. The additional constraint is that these 4 coefficients must
be normalize.

*Question 7* Display a configuration of the robot for which the norm of
the quaternion is bigger than one (e.g. 2.0). What happens?

During the search, the solver must respect this constraint. A solution
is to make this constraint explicit in the numerical program. However,
we will start by an easier quick-and-dirty trick. With quaternions, the
trick is simply to normalize any invalid quaternions. In the cost
function, first normalize the quaternion before computing the cost due
to the end-effector placement. An additional term should also be added
to the cost function to avoid excessive drift of the quaternion norm, in
particular with the norm going to 0.

*Question 8* Use +fmin\_bfgs+ to compute a configuration respecting a
given placement with the humanoid model, by normalizing the quaternion
at each step.

*Question 9* (harder) Do the same with the solver C-LSQ +fmin\_slsqp+,
with the explicit constraint that the norm of the quaternion must be 1.

Tutorial 2.5. Configuration of a parallel robot
-----------------------------------------------

A parallel robot is composed of several kinematic chains (called the
robot legs) that are all attached to the same end effector. This imposes
strict constraints in the configuration space of the robot: a
configuration is valide iff all the legs meets the same end-effector
placement. We consider here only the geometry aspect of parallel robots
(additionnally, some joints are not actuated, which causes additional
problems).

The kinematic structure of a paralel robot indeed induces loops in the
joint connection graph. In Pinocchio, we can only represents (one of)
the underlying kinematic tree. The loop constraints have to be handled
separately. An example that loads 4 manipulator arms is
link:tp2\_ur5x4\_py.html\[available here\]. Each leg i (for i=0,1,2,3)
of the robot is loaded in the list robots\[i\]. The loop constraints are
that the relative placement of every leg end-effector must stay the same
that in the initial configuration given as example in the above file.

*Question 10* Consider now that the orientation of the tool plate is
given by the quaternion Quaternion(0.7,0.2,0.2,0.6), with the
translation that you like. Find using the above optimization routines
the configuration of each robot leg so that the loop constraints are all
met.

//// Homework --------

Send by mail at nmansard@laas.fr a mail containing a single python file.
The subject of the mail should start with +\[SUPAERO\] TP2+ When
executed, the script should place the parallel robot toolplate at the
reference placement (whose rotation is Quaternion(0.7,0.2,0.2,0.6)) and
then move the legs to meet the loop constraints. ////
