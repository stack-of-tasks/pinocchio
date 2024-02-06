# Inverse kinematics (clik)

This example shows how to position the end effector of a manipulator robot to a given pose (position and orientation).
The example employs a simple Jacobian-based iterative algorithm, which is called closed-loop inverse kinematics (CLIK).

## Python
\includelineno inverse-kinematics.py

### Explanation of the code
First of all, we import the necessary libraries and we create the `Model` and `Data` objects:
\until data

The end effector corresponds to the 6th joint
\skipline JOINT

and its desired pose is given as
\skipline oMdes

Next, we define an initial configuration
\skipline q

This is the starting point of the algorithm. *A priori*, any valid configuration would work.
We decided to use the robot's neutral configuration, returned by algorithm `neutral`.
For a simple manipulator such as the one in this case, it simply corresponds to an all-zero vector,
but using this method generalizes well to more complex kinds of robots, ensuring validity.

Next, we set some computation-related values
\until damp
corresponding to the desired position precision (we will see later what it exactly means),
a maximum number of iterations (to avoid infinite looping in case the position is not reachable),
a positive "time step" defining the convergence rate,
and a fixed damping factor for the pseudoinversion (see below).

Then, we begin the iterative process.
At each iteration, we begin by computing the forward kinematics:
\skipline forwardKinematics

Next, we compute the error between the desired pose and the current one.
\skip dMi
\until err
Here, `data.oMi[JOINT_ID]` corresponds to the placement of the sixth joint (previously computed by `forwardKinematics`),
`dMi` corresponds to the transformation between the desired pose and the current one, and `err` is the error.
In order to compute the error, we employed `log`: this is a `Motion` object, and it is one way of computing an error in \f$SO(3)\f$ as a six-dimensional vector.

If the error norm is below the previously-defined threshold, we have found the solution and we break out of the loop
\until break
Notice that, strictly speaking, the norm of a spatial velocity does not make physical sense, since it mixes linear and angular quantities.
A more rigorous implementation should treat the linar part and the angular part separately.
In this example, however, we choose to slightly abuse the notation in order to keep it simple.

If we have reached the maximum number of iterations, it means a solution has not been found. We take notice of the failure and we also break
\until break

Otherwise, we search for another configuration trying to reduce the error.

We start by computing the Jacobian.
\skipline J

Next, we can compute the evolution of the configuration by solving the inverse kinematics.
In order to avoid problems at singularities, we employ the damped pseudo-inverse:
\f$v = - J^T (J J^T + \lambda I)^{-1} e\f$
implementing the equation as
\skipline v 
Notice that this way to compute the damped pseudo-inverse was chosen mostly because of its simplicity of implementation.
It is not necessarily the best nor the fastest way,
and using a fixed damping factor \f$\lambda\f$ is not necessarily the best course of action.

Finally, we can add the obtained tangent vector to the current configuration
\skipline q

where `integrate` in our case amounts to a simple sum. The resulting error will be verified in the next iteration.

At the end of the loop, we display the result:
\skip success
\until final

## C++
The equivalent C++ implemetation is given below

\includelineno inverse-kinematics.cpp

### Explanation of the code
The code follows exactly the same steps as Python.
Apart from the usual syntactic discrepancies between Python and C++, we can identify two major differences.
The first one concerns the Jacobian computation. In C++, you need to pre-allocate its memory space, set it to zero, and pass it as an input
\skip J(6,model.nv)
\until setZero

\skip computeJointJacobian
\line computeJointJacobian

This allows to always use the same memory space, avoiding re-allocation and achieving greater efficiency.

The second difference consists in the way the velocity is computed

\skip JJt
\until solve

This code is longer than the Python version, but equivalent to it. Notice we explicitly employ the `ldlt` decomposition.
