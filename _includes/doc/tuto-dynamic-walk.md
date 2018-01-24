<!-- MarkdownTOC -->

- [Objective](#objective)
- [Tutorial 4.1: defining input](#tutorial-41-defining-input)
- [Tutorial 4.2: computing reference ZMP](#tutorial-42-computing-reference-zmp)
- [Tutorial 4.3: reference trajectory of the center of mass](#tutorial-43-reference-trajectory-of-the-center-of-mass)
- [Tutorial 4.4: reference trajectories of the feet](#tutorial-44-reference-trajectories-of-the-feet)
- [Tutorial 4.5: generate walk motion](#tutorial-45-generate-walk-motion)

<!-- /MarkdownTC -->

<a name="objective"></a>

# Objective

The objective of this part is to generate a dynamically balanced walk motion for the humanoid robot as time-varying reference tasks for each foot and for the center of mass of the robot.
Tutorial 4.0: prerequesites

From the two previous tutorials, you should have the two following implementations available:

* A humanoid robot models, with at least two legs.
* An inverse geometry solver based on BFGS.

Yet, the inverse geometry only solves the motion of the robot for a constant task, like reaching a specific position of the hand, or a constant position of the center of mass.

It is possible to modify the BFGS call to perform an inverse kinematics by (i) limiting the number of iteration of BFGS to a small value e.g 10 iterations maximum, (ii) initializing the non-linear search from the previous configuration of the robot, and (iii) turning off the default verbose output of BFGS. For example, the robot can track a target moving vertically using the following example:

{% highlight python %}
cost.Mdes = se3.SE3( eye(3),np.matrix([0.2,0,0.1+t/100.]) ) # Reference target at time 0.
q = np.copy(robot.q0)
for t in range(100):
    cost.Mdes.translation = np.matrix([0.2,0,0.1+t/100.])
    q = a2m(fmin_bfgs(cost, m2a(q),maxiter=10,disp=False))
    robot.display(q)
{% endhighlight %}

Implement a motion of the right foot of the robot tracking a straight line from the initial position of the robot to a position 10cm forward, while keeping a constant rotation of the foot.

<a name="tutorial-41-defining-input"></a>

# Tutorial 4.1: defining input

The input of the walk generation algorithm is a sequence of steps with a given timing. This input will be represented by two sequences as in the examples below. The class `FootSteps` (see [code]({{ site.baseurl }}/assets/tutorials/footstep.html) ) can be used to define, store and access to the footstep plan.

{% highlight python %}
# Define 6 steps forward, starting with the left foot and stoping at the same forward position.
footsteps = FootSteps()
footsteps.initPositions( [0.0,-0.1] , [0.0,0.1] )
footsteps.addPhase( .3, 'none' )
footsteps.addPhase( .7, 'left' , [0.1,+0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [0.2,-0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , [0.3,+0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [0.4,-0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , [0.5,+0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [0.5,-0.1] )
footsteps.addPhase( .5, 'none' )
{% endhighlight %}

A phase _none_ defines a double support phase (no foot moving). A phase _left_ (resp. _right_) defines a simple support phase while indicating the flying foot. The time is a duration. The position is absolute.

Each interval corresponds to the following constant support phases:

<!-- tp4__1.png -->

{% highlight python %}
# Example of use
footsteps.getPhaseType(.4)         # return 'left'
footsteps.getLeftPosition(0.4)     # return 0,0.1
footsteps.getLeftNextPosition(0.4) # return 0.1,0.1
footsteps.getPhaseStart(0.4)       # return 0.3
footsteps.getPhaseDuration(0.4)    # return 0.7
footsteps.getPhaseRemaining(0.4)   # return 0.6
{% endhighlight %}

<a name="tutorial-42-computing-reference-zmp"></a>

# Tutorial 4.2: computing reference ZMP

Implement a python class called `ZmpRef` that takes as input a sequence of times and a sequence of steps. Objects of this class behave as a function of time that returns a 2 dimensional vector:

{% highlight python %}
zmp = ZmpRef (footsteps)
zmp (2.5)
array([ 0.41 ,  0.096])
{% endhighlight %}

The function should be a piecewise affine function

* starting in the middle of the ankles of the two first steps,
* finishing in the middle of the two ankles of the two last steps,
* constant under the foot support during single support phases.

You can use the template below.

{% highlight python %}
class ZmpRef (object):
    def __init__ (self, footsteps) :
        self.footsteps = footsteps
    # Operator ()
    def __call__ (self, t):
        return array (self.steps [0])
{% endhighlight %}

For the inputs provided above, the graph of _zmp_ is given below.

![zmp_ref against time]({{ site.baseurl }}/assets/tutorials/zmp-ref.png)

<a name="tutorial-43-reference-trajectory-of-the-center-of-mass"></a>

# Tutorial 4.3: reference trajectory of the center of mass

Using the reference zmp trajectory implemented in Tutorial 4.3, implement a class ComRef that computes the reference trajectory of the center of mass by optimal control.

To write the underlying optimization problem, you can use a factor graph. A simple implementation is available in (see [code]({{ site.baseurl }}/assets/tutorials/factor-graph.html) ). An example of use is the following. Try to guess the solution before executing it.

{% highlight python %}
f = FactorGraph(1,5)   # Define a factor of 5 variables of dimension 1

M = eye(1)             # M is simply 1 written as a 1x1 matrix.
for i in range(4):
    f.addFactorConstraint( [ Factor( i,M ), Factor( i+1,-M ) ], zero(1) )

f.addFactor( [ Factor(0,M) ], M*10 )
f.addFactor( [ Factor(4,M) ], M*20 )

x = f.solve()
{% endhighlight %}

<a name="tutorial-44-reference-trajectories-of-the-feet"></a>

# Tutorial 4.4: reference trajectories of the feet

Using the same method as in Tutorial 4.2, implement two classes RightAnkleRef and LeftAnkleRef that return reference positions of the ankles as homogeneous matrices. Unlike zmp reference, trajectories of the feet should be continuously differentiable.

<a name="tutorial-45-generate-walk-motion"></a>

# Tutorial 4.5: generate walk motion

Use the classes defined in the previous sections to generate a walk motion using the inverse kinematics solver of Tutorials 2 and 3.0.
