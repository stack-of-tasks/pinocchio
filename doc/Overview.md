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

\section OverviewIntro What is Pinocchio?

Pinocchio is a library for efficiently computing the dynamics (and derivatives) of a robot model, or of any articulated rigid-body model you would like (avatars in a simulator, skeletal models for biomechanics, etc.).
Pinocchio is one of the most efficient libraries for computing the dynamics of articulated bodies.
It implements the classical algorithms following the methods described in Featherstone's 2008
[book](http://www.springer.com/engineering/robotics/book/978-0-387-74314-1 "Rigid Body Dinamics Algorithms")
(many thanks to him).
It also introduces efficient variations of some of them, plus some new ones, notably including a full set of algorithms to compute the derivatives of the main ones.

Pinocchio is open-source, mostly written in C++ with Python bindings, and distributed under the BSD licence.
Contributions are welcome.

In this doc, you will find the usual description of the library functionalities, a quick tutorial to catch over the mathematics behind the implementation, a bunch of examples about how to implement classical applications (inverse kinematics, contact dynamics, collision detection, etc) and a set of practical exercices for beginners.

\section OverviewInstall How to install Pinocchio?

Pinocchio is best installed from APT packaging on Ubuntu 14.04, 16.04 and 18.04, from our repository.
On Mac OS X, we support the installation of Pinocchio through the Homebrew package manager.
If you just need the Python bindings, you can directly have access to them through Conda.
On systems for which binaries are not provided, installation from source should be straightforward.
Every release is validated in the main Linux distributions and Mac OS X.

The full installation procedure can be found on the Github Pages of the project:
http://stack-of-tasks.github.io/pinocchio/download.html.

\section OverviewSimple Simplest example with compilation command

We start with a simple program to compute the robot inverse dynamics. It is given in both C++ and Python version.

<table class="manual">
  <tr>
    <th>overview-simple.cpp</th>
    <th>overview-simple.py</th>
  </tr>
  <tr>
    <td valign="top">
      \include overview-simple.cpp
    </td>
    <td valign="top">
      \include overview-simple.py
    </td>
  </tr>
</table>

\subsection OverviewSimpleCompile Compiling and running your program

You can compile the C++ version by including Pinocchio and Eigen header directories.

\code g++ -I /path/to/eigen -I /path/to/pinocchio/include/ overview-simple.cpp -o overview-simple \endcode

where `/path/to/pinocchio` is your chosen installation directory for Pinocchio.
If you do not know Eigen's installation path, you can retrive it with `pkg-config --cflags eigen3`.
On Linux, it will usually be something like `/usr/include/eigen3/`.

Once your code is compiled, you might then run it using

\code ./overview-simple \endcode

In Python, just run it:

\code python overview-simple.py \endcode

\subsection OverviewSimpleExplain Explanation of the program

This program loads a robot model, creates a data structure for algorithm buffering, and runs the Recursive Newton-Euler Algorithm (RNEA) to compute the robot inverse dynamics.

We first include the proper files. In C++, there is not yet a rationalization of the include files. Here we have to include the sample model (where the parameters of the model we are using in the test are defined), and the header files defining the robot neutral position and the RNEA function. In Python, the library is easier to include by just importing pinocchio.

The first paragraph defines the model. In C++, we first define the object, then allocate it to be our sample model (a simple built-in manipulator, very useful for small tests). In Python, the model is created by calling a single command.
The model class contains the constant parameters of the robot (masses, segment length, body names, etc), i.e. the parameters that are not to be modified by the algorithms. As most algorithms need extra room for storage of internal values, we also have to allocate a Data structure, dedicated to the model. Pinocchio relies on a strict separation between constant parameters, in Model, and computation buffer, in Data.

Inverse dynamics computes the needed torque to track a trajectory defined by the joint configuration, velocity and acceleration.
In the second paragraph, we define these three quantities.
Velocity and acceleration are plain vectors, and can be initialized to any value.
Their dimension is `model.nv`, corresponding to the number of instantaneous degrees of freedom of the robot.
The configuration might be more complex than this (e.g. it can contain quaternions).
This is not the case in the current example, where we have a simple manipulator, but it might cause difficulties for more complicated models, such as humanoids.
Consequently, helper functions are provided to initialize a configuration to zero (neutral) or to a random value, or to make sure a configuration is valid (normalize).


Finally, we call the `rnea` function, by providing the robot model, the corresponding data, and the current configuration, velocity and acceleration.
The result is a vector of dimension `model.nv`. It is also stored in `data.tau`, in case it's needed.
For a manipulator, this value corresponds to the joint torque required to achieve the given motion. We just print it.
Notice that, in Python, we use numpy to represent matrices and vectors.
Therefore `tau` is `numpy` matrix object, and so are `q`, `v` and `a`.

Before we move on, a little tip: in our examples, we generaly include the whole namespace `pinocchio` for clarity.
However, if you feel like "pinocchio" is too long to type, the recommended shorthands are
```{.cpp}
namespace pin = pinocchio;
```
in C++
and
```{.py}
import pinocchio as pin
```
in Python.

\section OverviewComplex More complex example with C++ and Python

<table class="manual">
  <tr>
    <th>overview-urdf.cpp</th>
    <th>overview-urdf.py</th>
  </tr>
  <tr>
    <td valign="top">
      \include overview-urdf.cpp
    </td>
    <td valign="top">
      \include overview-urdf.py
    </td>
  </tr>
</table>

\subsection OverviewComplexCompile Compiling and running your program

This time, we must specify that URDFDOM is needed, as the model will be parsed from URDF.
Also, we need to link to `liburdfdom_model` and to `libboost_system`.

\code g++ -I /path/to/eigen -I /path/to/pinocchio/include/ -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_WITH_URDFDOM overview-urdf.cpp -lboost_system -lurdfdom_model -o overview-urdf \endcode

The program typically runs with a UR5 URDF description, that can be found for example in this repository https://github.com/humanoid-path-planner/ur_description

Now you can launch the program:
\code ./overview-urdf /path/to/ur5.urdf \endcode

In Python, just run it:

\code python overview-urdf.py /path/to/ur5.urdf \endcode

\subsection OverviewComplexExplain Explanation of the program

This program loads a model of a manipulator robot from a URDF file, computes the forward kinematics at a random initial configuration and displays the position of each robot joint with its name.

In C++, we need the headers of the model, the urdf parser, the random configuration and the forward-kinematics algorithms. In Python, only the Pinocchio module is needed.

The model is built by parsing the urdf model: the URDF file should be directly provided to the parser. Here we do not parse the geometries (yet), i.e. the shape of the bodies, but only the kinematic tree with names, lengths, and inertial properties. The Data object is then built from the Model object, like in the previous example. Next, we compute a random configuration and we print it.

The forward kinematics is simply computed by providing the model, the associated data and the robot configuration. Then, a loop goes through the kinematic tree (from root to leaves) and displays for each joint its name and position in the world frame. For the 6-DOF manipulator UR5, there are 7 joints to be displayed, with the first joint being the "universe", i.e. the reference world frame from which everything is expressed.

\section OverviewPython About Python wrappings

Pinocchio is written in C++, with a full template-based C++ API, for efficiency purposes. All the functionalities are available in C++. Extension of the library should be preferably in C++.

However, C++ efficiency comes with a higher work cost, especially for newcomers. For this reason, all the interface is exposed in Python. We tried to build the Python API as much as possible as a mirror of the C++ interface. The greatest difference is that the C++ interface is proposed using Eigen objects for matrices and vectors, that are exposed as NumPy matrices in Python.

When working with Pinocchio, we often suggest to first prototype your ideas in Python. Both the auto-typing and the scripting make it much faster to develop. Once you are happy with your prototype, then translate it in C++ while binding the API to have a mirror in Python that you can use to extend your idea.

\section OverviewCite How to cite Pinocchio

Happy with Pinocchio? Please cite us with the following format.

### Easy solution: cite our open access paper
The following is the preferred way to cite Pinocchio.
The paper is publicly available in HAL ([ref 01866228](https://hal.archives-ouvertes.fr/hal-01866228 "Pinocchio paper")).

\include carpentier-sii19.bib


### Citing the software package
Additionally, if you want to cite the software package, you might want to resort to the following citation.

\include pinocchioweb.bib

### Citing the derivative algorithms

A great novelty of Pinocchio is that the derivatives of rigid body dynamics algorithms are made publicly available for the first time.
If you want to refer to the new algorithms, without explicitley mentioning Pinocchio, you can cite

\include carpentier-rss18.bib


\section OverviewConclu Where to go from here?

This documentation is mostly composed of several examples and tutorials for newcomers, along with a technical documentation and a reference guide. If you want to make sure Pinocchio matches your needs, you may first want to check the list of features. Several examples in C++ and Python will then directly give you the keys to implement the most classical applications based on a rigid-body library. For nonexperts, we also provide the main mathematical fundamentals, based on Featherstone's formulations (you may prefer to buy and read the original book if you never did). A long tutorial in Python contains everything you need if you are not a Python expert and want to start with Pinocchio. This tutorial was first written as course material for a Master class about robotics. 

That's it for beginners. We then give an overview of the technical choices we made to write the library and make it efficient. Read this section if you want to extend the C++ library core, in particular if you are not familiar with the curiously-recursive template pattern (used for example in Eigen). A description of the benchmarks we did to test the library efficiency is also provided. 

By the way, Pinocchio has been deeply inspired by <a href="http://http://eigen.tuxfamily.org/">Eigen</a> (hence the structure of this page and the title of this last section). The API of our library is Eigen-based. If you are not comfortable with Eigen and want to use Pinocchio in C++, you better have to follow the <a href="http://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html">basic Eigen tutorials</a> first.










