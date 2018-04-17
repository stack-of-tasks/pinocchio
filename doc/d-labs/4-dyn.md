# 4) Snap your fingers (aka direct and inverse dynamics)

## Objectives

The main objective of the tutorial is to implement a simple torque
control inside a home-made contact simulator.

## 4.0) Technical prerequisites

### Robots

We are going to use a 4-finger hand, whose model is defined in Python
(no urdf model) using capsule volumes. The code of the robot is
[available here](robot__hand_8py_source.html). It needs a
`Display` class wrapping the Gepetto-viewer client [available here](display_8py_source.html),
and contains a `Robot` class implementing the robot hand and a simple example si [available
here](hand__example__8py_source.html).

```py
from robot_hand import Robot

robot = Robot()
robot.display(robot.q0)
```

Take care that the hand is small: zoom in to see it in the window (or
press the space bar).

### Solver

We will need a proper QP solver with inequality. QuadProg is a Python
wrap of a nice Golub-based solver. Install it with PIP

```
pip install --user quadprog
```

QuadProg main function is `solve_qp`. You have a bit of documentation
using the Python help command `help(solve_qp)`. A simple example
follows:

```py
from quadprog import solve_qp

# Solve min_x .5 xHx - gx s.t. Cx <= d
x, _, _, _, _, _ = solve_qp(H, g, C, d)
```

## 4.1. Direct dynamics

Choosing an arbitrary joint torque \f$\tau_q\f$, we now compute the robot
acceleration and integrate it.

The dynamic equation of the robot is \f$M a_q + b = \tau_q\f$, with \f$M\f$ the mass,
\f$a_q\f$ the joint acceleration and \f$b\f$ the drift. The mass matrix can be
computed using `CRB` algorithm (function of \f$q\f$). The drift is computed
using `RNE` algorithm (function of \f$q\f$, \f$v_q\f$ and \f$a_q\f$ with \f$a_q=0\f$).

```py
import pinocchio as se3

q = rand(robot.model.nq)
vq = rand(robot.model.nv)
aq0 = zero(robot.model.nv)
# compute dynamic drift -- Coriolis, centrifugal, gravity
b = se3.rnea(robot.model, robot.data, q, vq, aq0)
# compute mass matrix M
M = se3.crba(robot.model, robot.data, q)
```

These terms correspond to the inverse dynamics. They can be numerically
inverted to compute the direct dynamics.

#### Question 1
Using \f$M\f$ and \f$b\f$ computed by the above algorithms, and knowing
a given set of joint torques \f$\tau_q\f$, compute \f$a_q\f$ so that \f$M*a_q+b = \tau_q\f$.

Once \f$a_q\f$ as been computed, it is straight forward to integrate it to
velocity using \f$v_q += a_q * dt\f$. Integration to joint position is more
complex in general. It is implemented in pinocchio:

```py
q = se3.integrate(robot.model, q, vq * dt)
```

In the particular case of only simple joints (like the robot hand), the same integration
\f$q += v_q * dt\f$ also holds.

#### Question 2
Implement the simulation of the robot hand moving freely
with constant (possibly 0) torques. Implement a variation where the
torques are only joint friction (\f$\tau_q = -K_f v_q\f$ at each iteration).

## 4.2) PD and computed torques

Now choose a reference joint position (possibly time varying, like in
the hand example). The joint torques can then be computed to track the
desired position, with \f$\tau_q = -K_p (q-q_{des}) - K_v v_q\f$.
Both gains \f$K_p\f$ and \f$K_v\f$
should be properly chosen. Optimal tracking is obtained with
\f$K_v = 2 \sqrt{K_p}\f$. In general, a desired velocity is also tracked to avoid
tracking errors.

#### Question 3
Implement then simulate a PD, by compute the torques from a
PD law, then integrate it using the simulator of question 2.

Here, there is a strong coupling between joints, due to the mass matrix
that is not compensated in the simple PD law. In theory, the computed
torques is to compute the joint torque by inverse dynamics from a
reference joint acceleration. With boils down to canceling the
simulation equation by choosing the proper terms in the control law. It
is now very interesting to implement in case of perfect dynamics
knowledge. It might be more interesting to study in case the simulation
is done with the perfect M, while the control is computed with
approximate M (for example, using only the diagonal terms of the mass
matrix). Let's rather simulate contact.

## 4.3) Collision checking

The robot hand is composed of capsules, i.e. level-set of constant
distance to a segment. Collision checking and distances are then easy to
implement. The source code of collision checking is available in the
[robot_hand.py](robot__hand_8py_source.html) file.
Pinocchio also implement a complete and efficient
collision checking based on FCL, also not used in the tutorial.

Collision checking are done for a set of collision pairs that must be
specified to the robot. The collision checking method indeed compute the
distance between the two objects, along with the so-called witness
points. A method can also be used to display them.

```py
from robot_hand import Robot
robot = Robot()
robot.display(robot.q0)

# Create 10 witness points in the rendering window

for i in range(10):
    robot.viewer.viewer.gui.addCylinder('world/wa%i' % i, .01, .003, [1, 0, 0, 1])
    robot.viewer.viewer.gui.addCylinder('world/wb%i' % i, .01, .003, [1, 0, 0, 1])
    robot.viewer.viewer.gui.setVisibility('world/wa%i' % i, 'OFF')
    robot.viewer.viewer.gui.setVisibility('world/wb%i' % i, 'OFF')

# Add 4 pairs between finger tips and palm

robot.collisionPairs.append([2, 8])
robot.collisionPairs.append([2, 11])
robot.collisionPairs.append([2, 14])
robot.collisionPairs.append([2, 16])

# Compute distance between object 2 and 8, i.e the first collision pair

idx = 0
dist = robot.checkCollision(idx)

# Display the collision pair by adding two disks at the witness points.

robot.displayCollision(idx, 0)
```

The Jacobian of the corresponding pair can be computed using the
`collisionJacobian` method

```py
J = robot.collisionJacobian(idx, q)
```

The jacobian is a 1xN matrix (row
matrix) corresponding to the contact normal. Take care that some
information are stored in the visual objects when calling
checkCollision, that are later used by collisionJacobian. You have to
call collisionJacobian right after checkCollision, or the resulting
jacobian might not be coherent.

For all collision pairs in contact (distance below 1e-3), the Jacobian
must be collected and stacked in a single J matrix (which has as many
rows as active constraints). Similarly, distances must be stacked in a
vector (same number of rows as the jacobian).

Now, the joint acceleration is constrained by the contact constraint. It
can be written as a minimization problem using Gauss principle

\f$min \quad \frac{1}{2} (\ddot q - \ddot q_0 )^T M (\ddot q - \ddot q_0 )\f$

\f$s.t. \quad J \ddot q > 0 \f$

where \f$\ddot q_0\f$ is the free acceleration, i.e. the acceleration obtained
in Question 2 where no constraint is active.

In theory, the acceleration should be above the "centrifugal"
acceleration (i.e. the acceleration caused by joint velocity only, often
written \f$\dot J \dot q\f$) but we neglect it here.

In case of penetration or negative velocity, having only position
acceleration is not enough. A "trick" is often to require the contact
acceleration to be above a proportional depending of the penetration
distance: \f$J \ddot q >= -dist\f$, with \f$dist\f$ the vector of stacked
distances.

#### Question 4
Implement a contact simulator using QuadProg, the results
of Question 2 and the jacobian matrix of constraints whose distance is
below 1e-3.

A better solution to avoid penetration is to implement an impact model.
The simplest one is the inelastic impact, where normal velocity is
simply canceled at impact. For that, remember inactive contact (i.e.
those that were not in collision at previous simulation step). When a
collision pair is detected that was not previously active, project the
current velocity on the null space of all contacts:

\f$\dot q = \dot q - J^+ J \dot q\f$

#### Question 5

The complete loop should be as follows: \f$\tau_q\f$ is computed
from a PD tracking a time-varying joint position (question 3). After
computing \f$\tau_q\f$, all collision pairs must be checked to find those with
distances below 1e-3. Corresponding Jacobians must be computed and
stacked. If a new collision as appeared, the joint velocity must be
projected to nullify it. If not collision is active, the joint
acceleration is computed from inverting the mass matrix (question 2).
Otherwise, it is computed using QuadProg (question 4). The resulting
acceleration is integrated twice (question 1) before displaying the
robot starting a new simulation iteration.
