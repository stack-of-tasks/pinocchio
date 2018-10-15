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

This program loads a model of a manipulator robot from a URDF file, computes the forward kinematics at an arbitrary initial configuration and display the position of each robot joint with its name.

In C++, we need the headers of the model, the urdf parser and the forward-kinematics algorithms. In Python, Pinocchio and NumPy (numerics for Python) contexts are needed.

The model is built by parsing the urdf model: the .urdf file should be directly provided to the parser. Here we do not parse (yet) the geometries, i.e. the shape of the bodies, but only the kinematic tree with names, lengths, inertias. The Data object is then built from the Model object, like in the previous example.

The forward kinematics is simply computed by providing the model, the associated data and the robot configuration. Then, a loop goes through the kinematic tree (from root to leaves) and display for each joint its name and position in world frame. For the 6-dof manipulator UR5, there are 7 joints to be displayed, with the first joint being the "universe" i.e. the reference world frame from which everything is expressed.

# About Python wrapings

Pinocchio is written in C++, with a full template-based C++ API, for efficiency purpose. All the functionalities are available in C++. Extension of the library should be preferably in C++.

However, C++ efficiency comes with a higher work cost, especially for newcomers. For this reason, all the interface is exposed in Python. We tried to build the Python API as much as possible as a mirror of the C++ interface. The major difference is that the C++ interface is proposed using Eigen objects for matrices and vectors, that are exposed as Numpy Matrix in Python.

When working with Pinocchio, we often suggest to first prototype your ideas in Python. Both the auto-typing and the scripting make it much faster to develop. Once you are happy with your prototype, then translate it in C++ while binding the API to have a mirror in Python that you can use to extend your idea. 

# How to cite Pinocchio

Happy with Pinocchio? Please cite use with the following format.

### Easy solution: cite our paper available in HAL
The Pinocchio C++ library -- A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives, by  J. Carpentier, G. Saurel, G. Buondonno, J. Mirabel, F. Lamiraux, O. Stasse and N. Mansard, in HAL ref 01866228

\@article{carpentier-sii19, \n
&nbsp;&nbsp;  title={The Pinocchio C++ library--A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives},\n
&nbsp;&nbsp; author={J. Carpentier and G. Saurel and G. Buondonno and J. Mirabel and F. Lamiraux and O. Stasse and N. Mansard},\n
&nbsp;&nbsp; year={2018}\n
}


### Citing the software package

Pinocchio: fast forward and inverse dynamics for poly-articulated systems, 2015–2018, by J. Carpentier, J. Mirabel, N. Mansard et al, at https://stack-of-tasks. github.io/pinocchio

### Citing the derivative algorithms

Analytical derivatives of rigid body dynamics algorithms, 2018, by J. Carpentier and N. Mansard, in Robotics: Science and Systems

\@inproceedings{carpentier-rss18, \n
&nbsp;&nbsp;  title={Analytical derivatives of rigid body dynamics algorithms}, \n
&nbsp;&nbsp;  author={Carpentier, Justin and Mansard, Nicolas}, \n
&nbsp;&nbsp;  booktitle={Robotics: Science and Systems (RSS'18)}, \n
&nbsp;&nbsp;  year={2018}\n
}

# Presentation of the content of the documentation

This documentation is mostly composed of several examples and tutorials for newcomers, along with a technical documentation and a reference guide. If you want to make sure Pinocchio matches your needs, you may first want to check the list of features. Several examples in C++ and Python will then give you directly the keys to implement the most classical applications based on a rigid-body library. For nonexpert, we also provide the main mathematical fundamentals, based on Featherstone's formulations (you may prefer to buy and read the original book if you never did). A long tutorial in Python contains everything you need if you are not a Python expert and want to start with Pinocchio. This tutorial was first written as a class material for Master class about robotics.












