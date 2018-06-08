# 6) Take a walk (aka optimal control)

## Objective

The objective of this work is to get a first touch of optimal control
The tutorial will guide you to generate a dynamically balanced walk
motion for a humanoid robot using a LQR to compute the robot
center-of-mass trajectory.

## 6.0) prerequesites

### Prerequesite 1
A humanoid robot models, with at least two legs.

### Prerequesite 2
An inverse geometry solver based on BFGS.

Yet, the inverse geometry only solves the motion of the robot for a
constant task, like reaching a specific position of the hand, or a
constant position of the center of mass.

It is possible to modify the BFGS call to perform an inverse kinematics
by (i) limiting the number of iteration of BFGS to a small value e.g 10
iterations maximum, (ii) initializing the non-linear search from the
previous configuration of the robot, and (iii) turning off the default
verbose output of BFGS. For example, the robot can track a target moving
vertically using the following example:

```py
cost.Mdes = se3.SE3(eye(3), np.matrix([0.2, 0, 0.1 + t / 100.]))  # Reference target at time 0.
q = np.copy(robot.q0)
for t in range(100):
    cost.Mdes.translation = np.matrix([0.2, 0, 0.1 + t / 100.])
    q = fmin_bfgs(cost, q, maxiter=10, disp=False)
    robot.display(q)
```

Implement a motion of the right foot of the robot tracking a straight
line from the initial position of the robot to a position 10cm forward,
while keeping a constant rotation of the foot.

## 6.1) defining input

The input of the walk generation algorithm is a sequence of steps with a
given timing. This input will be represented by two sequences as in the
examples below. The class `FootSteps` [provided here](foot__steps_8py_source.html)
can be used to define, store and access to the footstep plan.

```py
# Define 6 steps forward, starting with the left foot and stoping at the same forward position.

footsteps = FootSteps([.0, -.1] ,[.0, .1])
footsteps.add_phase(.3, 'none')
footsteps.add_phase(.7, 'left', [.1, .1])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'right', [.2, -.1])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'left', [.3, .1])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'right', [.4, -.1])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'left', [.5, .1])
footsteps.add_phase(.1, 'none')
footsteps.add_phase(.7, 'right', [.5, -.1])
footsteps.add_phase(.5, 'none')
```

A phase 'none' defines a double support phase (no foot moving). A phase
'left' (resp. 'right') defines a simple support phase while indicating
the flying foot. The time is a duration. The position is absolute.

Each interval corresponds to the following constant support phases:

interval                            |                           |  
----------------------------------- | ------------------------- | ----------------------------------------------------
\f$\left[t_0,t_1\right]\f$          | double support phase      | left foot in steps [0], right foot in steps [1]
\f$\left[t_1,t_2\right]\f$          | left foot support phase   | right foot moving to steps [2]
\f$\left[t_2,t_3\right]\f$          | double support phase,     |  
\f$\left[t_3,t_4\right]\f$          | right foot support phase, | left foot moving to steps [3]
\f$\vdots\f$                        | \f$\vdots\f$              | \f$\vdots\f$
\f$\left[t_{m-2}, t_{m-1}\right]\f$ | double support phase,     | left foot in steps [p-2], right foot in steps [p-1]

```py
# Example of use

footsteps.get_phase_type(.4)  # return 'left'
footsteps.get_left_position(0.4)  # return 0,0.1
footsteps.get_left_next_position(0.4)  # return 0.1,0.1
footsteps.get_phase_start(0.4)  # return 0.3
footsteps.get_phase_duration(0.4)  # return 0.7
footsteps.get_phase_remaining(0.4)  # return 0.6
footsteps.is_double_from_left_to_right(0)  # return False
footsteps.is_double_from_left_to_right(1)  # return True
```

## 6.2) computing reference ZMP

Implement a python class called `ZmpRef` that takes as input a sequence
of times and a sequence of steps. Objects of this class behave as a
function of time that returns a 2 dimensional vector:

```py
zmp = ZmpRef(footsteps)
zmp(2.5)
array([0.41, 0.096])
```

The function should be a piecewise affine function

- starting in the middle of the ankles of the two first steps,
- finishing in the middle of the two ankles of the two last steps,
- constant under the foot support during single support phases.

You can use the template below.

```py
class ZmpRef(object):
    def __init__(self, footsteps):
        self.footsteps = footsteps

    def __call__(self, t):
        return np.array(self.footsteps[0])
```

For the inputs provided above, the graph of `zmp` is given below.


## 6.3) reference trajectory of the center of mass

Using the reference zmp trajectory implemented above,
implement a class `ComRef` that computes the reference trajectory of the
center of mass by optimal control.

To write the underlying optimization problem, you can use a factor
graph. A simple implementation is available in
[this file](factor_8py_source.html). An example of use is
the following. Try to guess the solution before executing it.

```py
f = FactorGraph(1, 5)  # Define a factor of 5 variables of dimension 1

M = eye(1)  # M is simply 1 written as a 1x1 matrix.
for i in range(4):
    f.add_factor_constraint([Factor(i, M), Factor(i + 1, -M)], zero(1))

f.addFactor([Factor(0, M)], M * 10)
f.addFactor([Factor(4, M)], M * 20)

x = f.solve()
```

## 6.4) reference trajectories of the feet

Using the same method as in 6.2, implement two classes
`RightAnkleRef` and `LeftAnkleRef` that return reference positions of
the ankles as homogeneous matrices. Unlike zmp reference, trajectories
of the feet should be continuously differentiable.

## 6.5) generate walk motion

Use the classes defined in the previous sections to generate a walk
motion using the inverse kinematics solver of Lab 2.
