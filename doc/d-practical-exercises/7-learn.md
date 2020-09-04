# 7) Learning to fly (aka policy learning)

## Objective

The objective of this tutorial is to study how to directly solve an
optimal control problem, either by computing a trajectory from current
state to goal, or by computing a policy. To keep decent computation
timings with simple python code, we will only work with a simple
inverted pendulum with limited torque, that must swing to raise to
standing configuration. The presented algorithms successively compute an
optimal trajectory, a discretized policy with a Q-table and with a
linear network, and a continuous policy with a deep neural network.

## 7.0) prerequesites

We need a pendulum model and a neural network.

### Prerequesite 1

Inverted pendulum model

Two models are provided. The first one is continuous and is implemented
with Pinocchio and is available in [pendulum.py](pendulum_8py_source.html)
with class `Pendulum`. The
code is generic for a N-pendulum. We will use the 1-dof model. The state
is `[q, v]` the angle and angular velocity of the pendulum. The control
is the joint torque. The pendulum weights 1kg, measures 1m with COM at
0.5m of the joint. Do not forget to start `gepetto-gui` before
rendering the model. Each time the simulator is integrated, it returns a
new state and the reward for the previous action (implement to be the
weighted sum of squared position, velocity and control). The state is
recorded as a member of the class and can be accessed through env.x. Do
not forget to copy it before modifying it.

```py
from pendulum import Pendulum
from pinocchio.utils import *

env = Pendulum(1)  # Continuous pendulum
NX = env.nobs      # ... training converges with q,qdot with 2x more neurones.
NU = env.nu        # Control is dim-1: joint torque

x = env.reset()    # Sample an initial state
u = rand(NU)       # Sample a control
x, reward = env.step(u)   # Integrate simulator for control u and get reward.
env.render()       # Display model at state env.x
```

A second version of the same model is provided with a discrete dynamics,
in [dpendulum.py](dpendulum_8py_source.html). The state is again `[q, v]`, however discretized in NQ
position and NV velocity. The state is then a integer equal to iq*NV+iv,
ranging from 0 to NQ*NV=NX. Controls are also discretized from 0 to NU.

```py
from dpendulum import DPendulum

env = DPendulum()
NX = env.nx  # Number of (discrete) states
NU = env.nu  # Number of (discrete) controls
env.reset()
env.step(5)
env.render()
```

Other models could be used. In particular, we used a similar API to the
Gym from OpenAI, that you might be interested to browsed and possibly
try with the following algorithms.

### Prerequesite 2
A neural network with optimizers

We will use the Tensor Flow from Google, available thanks to pip.

```
pip install --user tensorflow tflearn
```

## 7.1) Optimizing an optimal trajectory

For the first tutorial, we implement a nonlinear optimization program
optimizing the cost of a single trajectory for the continious pendulum.
The trajectory is represented by its initial state x0 and the vector of
piecewise-constant control `U=[u0 ... uT-1]`, with `T` the number of
timestep. The cost is simply the integral of the cost function l(x,u)
returned by the pendulum environment.

Then the integral cost is optimized starting from a 0 control
trajectory, until the pendulum finally reaches a standing state. Observe
that the number of swings is possibly sub-optimal.

The code to optimize the trajectory is available in [ocp.py](ocp_8py_source.html).

## 7.2) Q-table resolution for discrete pendulum

We now consider the discrete model, with NX state and NU control.
Imagine this model as a chess board with a maze (possibly nonplanar)
drawn on it, and you ask the system to discover a path from an inital
state to the final state at the center of the board. When performing a
trial, the system is informed of success or failure by receiving reward
1 when reaching the goal, or otherwise 0 after 100 moves. To record the
already-explored path, we can stored a table of NX per NU values, each
giving how likely we would be rewarded if taking action U at state X.
This table is named the Q-table, and corresponds to the Hamiltonian
(Q-value) of the discrete system. The Q-values can be back-propagated
along the table using the Dijkstra algorithm. Since we do not now the
goal(s) states, the back propagation is done along random roll-outs
inside the maze, which likely converges to an approximation of the exact
Hamiltonian. Once the Q-table is computed, the optimal policy is simply
chosen by maximizing the vector of Q-values corresponding to the row of
state X.

This algorithm is available in the file [qtable.py](qtable_8py_source.html).

## 7.3) Q-table using a linear net

The idea is to similarly approximate the Q-value for the continuous
model. Since the continious model has both infinitely many states and
controls, a table can not make it. We will rather use any function basis
to approximate the Q-value. For the tutorial, we have chosen to use a
deep neural net. Firt, let's use a simple net for storing the Q-table.

Basically, the vectory of Q-values for all possible control u is
obtained by multiplying the Q-table by a one-hot vector (0 everywhere
except a single 1) corresponding to the state. The optimal policy is
then the maximum of this vector: `iu^* = argmax(Q*h(ix))`, with `h(ix)
= [ 0 0 ... 0 1 0 ... 0]`, ix and iu being indexes of both state and
control. We use tensor flow to store array Q. The Q-value net is simply
the multiplication of Q by one-hot x, and the policy the argmax of the
result.

Now, the coefficients of Q are the parameters defining the Q-value (and
then the policy). They must be optimized to fit the cost function. From
Hamiltion-Jacobi-Belman equation, we know that Q(x,u) = l(x,u) + max\_u2
Q(f(x,u),u2). We optimize the Q value so that this residual is minimized
along the samples collected from successive roll-outs inside the maze.

The implementation of this algorithm is available in [qnet.py](qnet_8py_source.html).
Observe that the convergence is not as fast as with the Q-Table algorithm.

## 7.4) Actor-critic network

We will now optimize a continuous policy and the corresponding
Q-function, using an "Actor-Critic" method proposed in ["Continuous
control with deep reinforcement learning", by Lillicrap et al,
arXiv:1509.02971](https://arxiv.org/abs/1509.02971).

Two networks are used to represent the Q function and the policy. The
first network has two inputs: state x and control u. Its outpout is a
scalar. It is optimized to minimize the residual corresponding to HJB
equation along a batch of sample points collected along previous
roll-outs.

The policy function has a single input: state X. Its output is a control
vector U (dimension 1 for the pendulum). It is optimize to maximize the
Q function , i.e at each state, Pi(x) corresponds to the maximum over
all possible controls u of Q(x,u).

Two critical aspects are reported in the paper and implemented in the
tutorial. First, we learn over a batch of random samples collected from
many previous roll-outs, in order to break the temporal dependancy in
the batch. Second, we regularize the optimization of both Q-value
(critic) and policy (actor) networks by storing a copy of both network,
and only slightly modifying these copy at each steps.

The corresponding algorithm is implemented in the file [continuous.py](continuous_8py_source.html)
The training phase requires 100 roll-outs and some
minutes (maybe more on a virual machine).

## 7.5) Training the network with the OCP solver

Using the OCP solver, you might compute a few optimal trajectories (say
10) starting from various initial conditions. Initialize the replay
memory with the 10x50 points composing the 10 optimal trajectories and
optimize the network from these replay memory only (without additional
roll-outs, but using the same small-size batch). Play with the learning
parameters until the network converges.

When properly implemented, the OCP produces better accuracy than the
policy. However, at run-time, the policy is much cheaper to evaluate
than solving a new OCP. I am currently considering how to use the
network to warm-start or guide the OCP solver at run-time.

The provided solvers (trajectory and policy) runs reasonably well for
the 1-pendulum. It is more difficult to tune for a more-complex
dynamics, such as a 2-pendulum. You may want to try on a quadcopter
robot (hence the title of the tutorial) but I except it to be a serious
job.
