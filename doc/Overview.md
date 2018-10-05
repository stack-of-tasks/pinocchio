# Overview {#index}
<!--
//
// Copyright (c) 2016, 2018 CNRS
// Author: Florent Lamiraux, Justin Carpentier, Guilhem Saurel
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.
-->

# What is Pinocchio?

Pinocchio is a library for efficiently computing the dynamics (and derivatives) of a robot model, and behing, any articulated rigid-body model you would like (avatar in a simulator, skeletal model for biomechanics).
Pinocchio is one of the most efficient library for computing the dynamics of articulated bodies.
It implements the classical algorithms following the methods described in Featherstone 2009 book (many thanks to him).
It also introduce efficient variations of some of these algorithms, some new ones and algorithms to compute the derivatives of the main algorithms.

Pinocchio is open-source, mostly written in C++ with Python bindings, and distributed under LGPL-v3 licence.
Contributions are welcome.

In this doc, we will find the usual description of the library functionalities, a quick tutorial to catch over the mathematics behind the implementation, a bunch of examples about how to implement classical applications (inverse kinematics, contact dynamics, collision detection, etc) and a set of practical exercices for beginners.

# How to install Pinocchio?

Pinocchio is best installed from APT packaging on Ubuntu 14.04 and 16.04, from our repository.
Every released is validated in the main Linux distribution and OSX, for which installation from source should be straightforward.

The full installation procedure can be found on the Github Pages of the project:
http://stack-of-tasks.github.io/pinocchio/download.html.

# Simplest example with compilation command

We start by a simple program to compute the robot inverse dynamics. It is given in both C++ and Python version.

<table class="manual">
<tr><th>C++ version</th><th>Python version</th></tr>
<tr><td>
\include overview-simple.cpp
</td>
<td>
\include overview-simple.py
</td></tr></table>

## Compiling and running your program

You would compile the C++ version by including Pinocchio and Eigen header directories.

\code g++ -I /usr/include/eigen -I /path/to/pinocchio/include sample-rnea.cpp -L /path/to/pinocchio/lib -lpinocchio -o sample-rnea \endcode

You might then run it using

\code ./sample-rnea \endcode

In Python, just run it:

\code python sample-rnea.py \endcode

## Explaination of the program

This program load a robot model, create a data structure for algorithm buffering, and run the Recursive-Newton-Euler Algorithm (rnea) to compute the robot inverse dynamics.

We first include the proper files. In C++, there is not yet an rationalization of the include files. Here we have to include the sample model (where the parameters of the model we are using in the test are defined), and the header files defining the robot neutral position and the RNEA function. In Python, the library is easier to include by just importing pinocchio.

The first paragraph defines the model. In C++, we first define the object, then allocate it to be our sample model (a human-like kinematic tree with random parameters, very useful for small tests). In Python, the model is created while calling the sample humanoid model.
The model class contains the constant parameters of the robot (masses, segment length, body names, etc), i.e. the parameters that are not to be modified by the algorithms. As most algorithms need extra room for storage of internal values, we also have to allocate a Data structure, dedicated to the model. Pinocchio relies on a strict separation between constant parameters, in Model, and calculus buffer, in Data.

Inverse dynamics computes the needed torque to track a trajectory defined by position, velocity and acceleration in the configuration space.
Velocity and acceleration are plain vectors, and can be initialized to any value.
The configuration might be more complex (e.g. contains quaternion).
Consequently, helper functions are provides to initialized to zero (neutral) or random configuration, or make sure a configuration is valid (normalize).

Finally, we call the rnea function, by providing the robot model, the corresponding data, and the current configuration, velocity and acceleration.
The result is a vector of dimension model.nv. It is also stored in data.tau if needed.
We just print it, also the corresponding values are difficult to interpret.

## More complex example with C++ & Python

<table class="manual">
<tr><th>C++ version</th><th>Python version</th></tr>
<tr><td>
\include overview-urdf.cpp
</td>
<td>
\include overview-urdf.py
</td></tr></table>

## Compiling and running your program

This time, we must specify that URDFDOM is needed, as the model will be parsed from URDF.

\code g++ -I /usr/include/eigen3/  -I  /path/to/pinocchio/include -L /path/to/pinocchio/lib -DURDFDOM_TYPEDEF_SHARED_PTR -DWITH_URDFDOM  ../examples/sample-urdf.cpp  -lpinocchio -o sample-urdf \endcode

The program typically runs with a UR5 URDF description, that can be found for example in this repository https://github.com/humanoid-path-planner/universal_robot

Launch the program from the directory containing the ur5.urdf file
\code cd /path/to/ur5.urdf && ./sample-urdf \endcode

In Python, just run it:

\code python sample-urdf \endcode

## Explaination of the program

This program loads a model of a manipulator robot from a URDF file, decides an arbitrary initial configuration and desired effector position, and performs an inverse kinematics to reach this target.

In C++, we need the header of the urdf parser, and of algorithms related to jacobian and joint-configuration computations.


## About Python wrapings

TODO philosophy and example

## How to cite Pinocchio

TODO: ref

## Presentation of the content of the documentation








