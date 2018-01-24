<!-- MarkdownTOC -->

- [Objectives](#objectives)
- [Tutorial 2.0. Technical prerequisites](#tutorial-10-technical-prerequisites)
  - [Robots](#robots)
- [Tutorial 2.1. Position the end effector](#tutorial-11-position-the-end-effector)
- [Tutorial 2.2. Approaching the redundancy](#tutorial-12-approaching-the-redundancy)
- [Tutorial 2.3. Placing the end-effector](#tutorial-13-placing-the-end-effector)
- [Tutorial 2.4. Working with a mobile robot](#tutorial-14-working-with-a-mobile-robot)

<!-- /MarkdownTC -->

<a name="objectives"></a>

# Objectives

The main objective of the first tutorial is to compute a configuration of the robot minimizing a cost (maximizing a reward) and respecting some given constraints. Additionally, the objectives are to use a robot model in Pinocchio, optimally the models that were built during the first tutorial, to have a first hands on the difficulties of working outside of real vector spaces and to consider what are the guesses that are taken by an optimal

<a name="tutorial-10-technical-prerequisites"></a>

# Tutorial 2.0. Technical prerequisites
Python SciPy and MatplotLib

You will need the two libraries Python SciPy (scientific Python) and MatPlotLib (plot mathematical data).

SciPy can be installed by `sudo apt-get install python-scipy`. Alternatively, if you do not have access to the network, you can directly install the three following packages:

- [libamd.deb](http://homepages.laas.fr/nmansard/teach/robotics2015/libamd2.2.0_i386.deb)
- [libumfpack.deb](http://homepages.laas.fr/nmansard/teach/robotics2015/libumfpack5.4.0_i386.deb)
- [scipy.deb](http://homepages.laas.fr/nmansard/teach/robotics2015/python-scipy_0.9.0_i386.deb)

Download the three files, and then type:

    student@pinocchio1204x32vbox:~$ sudo dpkg -i libamd2.2.0_i386.deb
    student@pinocchio1204x32vbox:~$ sudo dpkg -i libumfpack5.4.0_i386.deb
    student@pinocchio1204x32vbox:~$ sudo dpkg -i python-scipy_0.9.0_i386.deb


MatPlotLib can be installed by `sudo apt-get install python-matplotlib`. It is already installed on the VirtualBox.

We will use SciPy for its optimization toolbox. In the version 0.9 installed in the VirtualBox, the two most interesting solvers are BFGS and a constrained least square. Documentation on both solvers is available here:

* General documentation: [SciPy.optimize](http://docs.scipy.org/doc/scipy-0.9.0/reference/tutorial/optimize.html)
* Manual of BFGS: [fmin_bfgs](http://docs.scipy.org/doc/scipy-0.9.0/reference/generated/scipy.optimize.fmin_bfgs.html#scipy.optimize.fmin_bfgs)
* Manual of constrained least square: [fmin_slsqp](http://docs.scipy.org/doc/scipy-0.9.0/reference/generated/scipy.optimize.fmin_slsqp.html#scipy.optimize.fmin_slsqp)

Examples of calls of these two functions are given below. We will use both solvers with numerical (finite-differencing) differenciation, to avoid the extra work of differencing the cost and constraint functions by hand. In general, it is strongly advice to first test a numerical program with finite differencing, before implementing the true derivatives only if needed. In any case, the true derivatives must always be checked by comparing the results with the finite differenciation.

Additionally, the provided implementation of BFGS allows the user to provide a callback function and track the path taken by the solver, but does not provide the possibility to specify constraints (constraints can be added as penalty functions in the cost, but this requires additional work). The constrained least-square implementation allows the user to specify equality and inequality constraints, but not the callback. In the following, start to use BFGS before moving to the constrained least-square only when constraints are really needed.

{% highlight python %}
# Example of use a the optimization toolbox of SciPy.
import numpy as np
from scipy.optimize import fmin_bfgs, fmin_slsqp

def cost(x):
     '''Cost f(x,y) = x^2 + 2y^2 - 2xy - 2x '''
     x0 = x[0]
     x1 = x[1]
     return -1*(2*x0*x1 + 2*x0 - x0**2 - 2*x1**2)

def constraint_eq(x):
     ''' Constraint x^3 = y '''
     return np.array([ x[0]**3-x[1] ])

def constraint_ineq(x):
     '''Constraint x>=2, y>=2'''
     return np.array([ x[0]-2,x[1]-2 ])

class CallbackLogger:
     def __init__(self):
          self.nfeval = 1
     def __call__(self,x):
          print '===CBK=== {0:4d}   {1: 3.6f}   {2: 3.6f}'.format(self.nfeval, x[0], x[1], cost(x))
          self.nfeval += 1

x0 = np.array([0.0,0.0])
    # Optimize cost without any constraints in BFGS, with traces.
xopt_bfgs = fmin_bfgs(cost, x0, callback=CallbackLogger())
print '\n *** Xopt in BFGS = ',xopt_bfgs,'\n\n\n\n'

# Optimize cost without any constraints in CLSQ
xopt_lsq = fmin_slsqp(cost,[-1.0,1.0], iprint=2, full_output=1)
print '\n *** Xopt in LSQ = ',xopt_lsq,'\n\n\n\n'

# Optimize cost with equality and inequality constraints in CLSQ
xopt_clsq = fmin_slsqp(cost,[-1.0,1.0],
                       f_eqcons=constraint_eq, f_ieqcons=constraint_ineq,
                       iprint=2, full_output=1)
print '\n *** Xopt in c-lsq = ',xopt_clsq,'\n\n\n\n'
{% endhighlight %}

Take care that all _SciPy_ always works with vectors represented as 1-dimensional array, while Pinocchio works with vectors represented as matrices (which are in fact two-dimensional arrays, with the second dimension being 1). You can pass from a SciPy-like vector to a Pinocchio-like vector using:

{% highlight python %}
import numpy as np
x = np.array([ 1.0, 2.0, 3.0 ])
q = np.matrix(x).T
x = q.getA()[:,0]
{% endhighlight %}

The second library MatPlotLib plots values on a 2D graph. Once more, documentation is available here: URLTODO. An example is provided below.

{% highlight python %}
import numpy as np
import matplotlib.pyplot as plt
# In plt, the following functions are the most useful:
#    ion,plot,draw,show,subplot,figure,title,savefig

# For use in interactive python mode (ipthyon -i)
interactivePlot = False

if interactivePlot:
    plt.ion() # Plot functions now instantaneously display, shell is not blocked


# Build numpy array for x axis
x = 1e-3 * np.array (range (100))
# Build numpy array for y axis
y = x**2

fig = plt.figure ()
ax = fig.add_subplot ('111')
ax.plot (x, y)
ax.legend (("x^2",))

if not interactivePlot:
    # Display all the plots and block the shell.
    # The script will only ends when all windows are closed.
    plt.show ()
{% endhighlight %}

<a name="robots"></a>

## Robots

We will need two models of robots: one fixed serial-chain manipulator robot with 7-DOF and only simple joints (i.e. having their configuration in a real vector space, e.g. prismatic or revolute); and one mobile "free-floating" robot (i.e. with the first joint being a "JointModelFreeFlyer", with the configuration containing a quaternion).

The robot models are assumed to be accessible through two Python classes named RobotFixed for the fixed serial-chain robot, and RobotFloating for the mobile robot. The following operations are supposed to be implemented:

{% highlight python %}
import pinocchio as se3
robot = RobotFixed()       # Possibly set arguments in the constructor.
print robot.model          # Access to the object se3.Model.
print robot.data           # Access to the object se3.Data.
print robot.viewer         # (or robot.display.viewer)  ...
                           # ... Access to the object gepetto.corbaserver.client.
q = robot.q0               # Access to an arbitrary reference position.
robot.display(q)           # Display the robot in Gepetto-Viewer.
i = robot.indexEffector    # Access to the index of the  (or to one of the) robot effector.
se3.geometry(robot.model,robot.data,q)
print robot.data.oMi[i]    # Access to the placement of the end effector at configuration q.
# The three last lines might be embedded in a single method Robot.Mendeffector(q).
{% endhighlight %}
These two models are the output of the first tutorial. The most interesting option is to use them.

Alternatively, a RobotWrapper Class is available directly with Pinocchio.  You can use it to load a model from a urdf file and automatically get the visual meshes described in this urdf. The RobotWrapper Class is able then to automatically communicate with the Gepetto viewer in order to display your robts.

Two urdf models are available and are described below.

Baxter is a fixed robot with two arms and a head developed by the USA company RethinkRobotics. All its 15 joints are revolute joints. Its configuration is in R^15 and is not subject to any constraint. The model of Baxter is described in a URDF file, with the visuals of the bodies of the robot being described as meshed (i.e. polygon soups) using the Collada format ".dae". Both the URDF and the DAE files are available in the attached ZIP archive. Uncompressed it in the VirtualBox, for example in the directory "/home/student/src/pinocchio/models".

. Baxter model and code baxter.zip


Romeo is a humanoid robot developed by the French-Japanese company Aldebaran Robotics. It has two legs, two arms and a head, for a total of 31 joints (plus 6DOF on the free flyer). Its model is natively in Pinocchio (no additional dowload needed). Romeo can be loaded with:



Additionally, the index of right and left hands and feet are stored in romeo.rh, romeo.lh, romeo.rf and romeo.lf.

The RobotWrapper Class is the generic one that we advice you to use for your own urdf files. If you decide to use the model of Romeo, provided with the source files of Pinocchio, you can use the RomeoWrapper Class that is a slight specialisation of the RobotWrapper for this model in particular. You can find shortcuts to particular joints such as `romeo.rh` for the right hand, and more.

To create a robot with these wrappers, you can do as follow


{% highlight python %}

import pinocchio as se3
import numpy as np

from pinocchio.robot_wrapper import RobotWrapper

hint_list = ["/local/devel/src/pinocchio/models","other/path"]
robot = RobotWrapper("/local/devel/src/pinocchio/models/romeo.urdf",hint_list, se3.JointModelFreeFlyer())

robot.initDisplay()
robot.loadDisplayModel("world/pinocchio")

q0 = np.matrix([
    0, 0, 0.840252, 0, 0, 0, 1,  # Free flyer
    0, 0, -0.3490658, 0.6981317, -0.3490658, 0,  # left leg
    0, 0, -0.3490658, 0.6981317, -0.3490658, 0,  # right leg
    0,  # chest
    1.5, 0.6, -0.5, -1.05, -0.4, -0.3, -0.2,  # left arm
    0, 0, 0, 0,  # head
    1.5, -0.6, 0.5, 1.05, -0.4, -0.3, -0.2,  # right arm
]).T

robot.display(q0)

{% endhighlight %}

The Class `robotWrapper` will call for you the urdf parser and the geometry parser available in the C++ part of Pinoccchio. To do that, you need to provide the full path to the urdf file to create the kinematic chain. If you work with visuals and collisions, you then need to specify where to search for the meshes. The urdf format specifies meshes path as `package://path/to/mesh.dae`. Our parser works with absolute path so you need to tell him the directories that will be replacing the _packages://_ part. If you don't specify any hint list, it will look in the environment variable `ROS_PACKAGE_PATH`. If you specify a list, it will search in the first path and then search in the second only if no meshes are found in the first, etc..


<a name="tutorial-11-position-the-end-effector"></a>

# Tutorial 2.1. Position the end effector

The first tutorial is to position (i.e. translation only) the end effector of a manipulator robot to a given position. For this first part, we will use the fixed serial-chain robot model.

Recall first that the position (3D) of the joint with index "i" at position "q" can be access by the following two lines of code:

{% highlight python %}
# Compute all joint placements and put the position of joint "i" in variable "p".
import pinocchio as se3
se3.geometry(robot.model,robot.data,q)
p = robot.data.oMi[i].translation
{% endhighlight %}

__Question 1:__ Using this, build a cost function a cost function as the norm of the difference between the end-effector position `p` and a desired position `pdes`. The cost function is a function that accepts as input an 1-dimensional array and return a float.

__Question 2:__ Then use `fmin_bfgs` to find a configuration q with the end effector at position `pdes`.

__Question 3:__ Finally, implements a callback function that display in Gepetto-Viewer every candidate configuration tried by the solver.

<a name="tutorial-12-approaching-the-redundancy"></a>

# Tutorial 2.2. Approaching the redundancy

The manipulator arm has 7 DOF, while the cost function only constraints 3 of them (the position of the end effector). A continuum of solutions then exists. The two next questions are aiming at giving an intuition of this continuum.

__Question 4:__ Sample several configurations respecting `pdes` by giving various initial guesses to the solver. Store this sampling of solutions in a list, then display this list in Gepetto-Viewer, each configuration begin displayed during 1 second (pause of 1 seconds can be obtained using: import time; time.sleep(1.0)).

A configurations in this continuum can then be selected with particular properties, like for example being the closest to a reference configuration, or using some joints more than the others, or any other criterion that you can imagine.

__Question 5:__ Sum a secondary cost term to the first positioning cost, to select the posture that maximizes the similarity (minimizes the norm of the difference) to a reference posture. The relative importance of the two cost terms can be adjusted by weighting the sum: find the weight so that the reference position is obtained with a negligible error (below millimeter) while the posture is properly taken into account.

<a name="tutorial-13-placing-the-end-effector"></a>

# Tutorial 2.3. Placing the end-effector

The next step is to find a configuration of the robot so that the end effector respects a reference placement, i.e. position and orientation. The stake is to find a metric in SE(3) to continuously quantify the distance between two placements. There is no canonical metric in SE(3), i.e. no absolute way of weighting the position with respect to the orientation. Two metrics can be considered, namely the log in SE(3) or in R^3 x SE(3). The tutorial will guide you through the first choice.

The SE(3) and SO(3) logarithm are implemented in Pinocchio in the explog module.

{% highlight python %}
from pinocchio.explog import log
from pinocchio import SE3
nu = log(SE3.Random())
nu_vec = nu.vector()
{% endhighlight %}
```
__Question 6:__ Solve for the configuration that minimizes the norm of the logarithm of the difference between the end effector placement and the desired placement.

Optionally, try other metrics, like the log metric of R^3 x SO(3), or the Froebenius norm of the homogeneous matrix.

<a name="tutorial-14-working-with-a-mobile-robot"></a>

# Tutorial 2.4. Working with a mobile robot

Until now, the tutorial only worked with a simple manipulator robot, i.e. whose configuration space is a real vector space. Consider now the humanoid robot, whose first joint is a free joint: it has 6 degrees of freedom (3 rotations, 3 translations) but its configuration vector is dimension 7. You can check it with `robot.model.nq`, that stores the dimension of the configuration, and `robot.model.nv`, that stores the dimension of the configuration velocity, i.e. the number of degrees of freedom. For the humanoid, nq = nv+1.

Indeed, the configuration coefficients 3 to 7 are indeed representing a quaternion. The additional constraint is that these 4 coefficients must be normalize.

__Question 7:__ Display a configuration of the robot for which the norm of the quaternion is bigger than one (e.g. 2.0). What happens?

During the search, the solver must respect this constraint. A solution is to make this constraint explicit in the numerical program. However, we will start by an easier quick-and-dirty trick. With quaternions, the trick is simply to normalize any invalid quaternions. In the cost function, first normalize the quaternion before computing the cost due to the end-effector placement. An additional term should also be added to the cost function to avoid excessive drift of the quaternion norm, in particular with the norm going to 0.

__Question 8:__ Use `fmin_bfgs` to compute a configuration respecting a given placement with the humanoid model, by normalizing the quaternion at each step.

__Question 9:__ Do the same with the solver C-LSQ `fmin_slsqp`, with the explicit constraint that the norm of the quaternion must be 1.
