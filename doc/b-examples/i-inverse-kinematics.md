# Inverse kinematics (clik)

This example shows how to position the end effector of a manipulator robot to a given position.
The example employs a simple Jacobian-based iterative algorithm, which is called closed-loop inverse kinematics (CLIK).

## Python
\includelineno i-inverse-kinematics.py

### Explanation of the code
First of all, we import the necessary libraries and we create the `Model` and `Data` objects:
\until data

The end effector corresponds to the 6th joint
\skipline JOINT

and its desired position is given as
\skipline xdes

Next, we define an initial configuration
\skipline q

This is the starting point of the algorithm. *A priori*, any valid configuration would work.
We decided to use the robot's neutral configuration, returned by algorithm `neutral`.
For a simple manipulator such as the one in this case, it simply corresponds to an all-zero vector,
but using this method generalizes well to more complex kinds of robots, ensuring validity.

Next, we set some computation-related values
\until DT
corresponding to the desired position precision (a tenth of a millimeter will do), a maximum number of iterations (to avoid infinite looping in case the position is not reachable) and a positive "time step" defining the convergence rate.

Then, we begin the iterative process.
At each iteration, we begin by computing the end-effector pose for the current configuration value:
\skip forwardKinematics
\until R

Next, we compute the error between the desired position and the current one. Notice we chose to express it in the local joint frame:
\skipline err

If the error norm is below the previously-defined threshold, we have found the solution and we break out of the loop
\until break

If we have reached the maximum number of iterations, it means a solution has not been found. We print an error message and we also break
\until break

Otherwise, we search for another configuration trying to reduce the error.

We start by computing the Jacobian, also in the local joint frame. Since we are only interested in the position, and not in the orientation, we select the first three lines, corresponding to the translation part
\skipline J

Next, we can compute the evolution of the configuration by taking the pseudo-inverse of the Jacobian:
\skipline v

Finally, we can add the obtained tangent vector to the current configuration
\skipline q

where `integrate` in our case amounts to a simple sum. The resulting error will be verified in the next iteration.

At the end of the loop, we display the result:
\skip result
\until final

## C++
The equivalent C++ implemetation is given below

\includelineno i-inverse-kinematics.cpp

### Explanation of the code
The code follows exactly the same steps as Python.
Apart from the usual syntactic discrepancies between Python and C++, we can identify two major differences.
The first one concerns the Jacobian computation. In C++, you need to pre-allocate its memory space, set it to zero, and pass it as an input
\skipline J(6,model.nv)
\skipline jointJacobian

This allows to always use the same memory space, avoiding re-allocation and achieving greater efficiency.

The second difference consists in the way the velocity is computed

\dontinclude i-inverse-kinematics.cpp
\skip svdOptions
\until BDCSVD
\skipline svd.compute

This is equivalent to using the pseudo-inverse, but way more efficient.
Also notice we have chosen to pre-allocate the space for the SVD decomposition instead of using method `bdcSvd(svdOptions)`.

