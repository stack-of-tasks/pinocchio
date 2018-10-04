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

# Installation

Pinocchio is best installed from APT packaging on Ubuntu 14.04 and 16.04, from our repository.
Every released is validated in the main Linux distribution and OSX, for which installation from source should be straightforward.

The full installation procedure can be found on the Github Pages of the project:
http://stack-of-tasks.github.io/pinocchio/download.html.

# Simplest example with compilation command

## C++
\include overview-simple.cpp

## Python
\include overview-simple.py




## More complex example with C++ & Python

TODO: load model and run loop for an IK with 3D task


## About Python wrapings

TODO philosophy and example

## How to cite Pinocchio

TODO: ref

## Presentation of the content of the documentation








