\page md_doc_a-features_g-dynamic Dynamics algorithms

## Inverse dynamics

The Recursive Newton-Euler Algorithm (RNEA) computes the inverse dynamics:
given a desired robot configuration, velocity and acceleration, the torques
required to execute this motion are computed and stored. The algorithm first
performs a forward pass (equivalent to second-order kinematics). It then
performs a backward pass, computing the wrenches transmitted along the
structure and extracting the joint torques needed to obtain the computed link
motions. With the appropriate inputs, this algorithm can also be employed to
compute specific terms of the dynamic model, such as the gravity effects.

## Joint-space inertia matrix

The Composite Rigid Body Algorithm (CRBA) is employed to compute the joint
space inertia matrix of the robot. We have implemented some slight
modifications of the original algorithm that improve the computational
efficiency.

## Forward dynamics

The Articulated Body Algorithm (ABA) computes the unconstrained forward
dynamics: given a robot configuration, velocity, torque and external forces,
the resulting joint accelerations are computed.

## Additional algorithms

Beside the algorithms above, other methods are provided, most notably for
constrained forward dynamics, impulse dynamics, inverse of the joint space
inertia and centroidal dynamics.
