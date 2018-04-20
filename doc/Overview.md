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


This is the documentation of Pinocchio, the Eigen-like library for Rigid Body Dynamics computations.
This library implements highly efficient kinematic and dynamic algorithms for multi-body systems making Pinocchio a versatile framework for robotics applications.

## Installation

The full installation procedure can be found on the Github Pages of the project:
http://stack-of-tasks.github.io/pinocchio/download.html.

## Introduction

One of the main features is the separation between models (constant values representing an object) and data (used for intermediate computations).
The two main classes are:
- se3::Model that represents a kinematic chain composed of joints that move links with mass and inertia,
- se3::Data that stores intermediate data for kinematic and dynamic computations.

### Spatial Algebra

The library provides some classes to represent spatial quantities at the root of kinematic and dynamic computations. This main classes are reported below:
- se3::SE3Tpl represents a rigid placement, a mathematical representation of \f$ SE(3) \f$.
- se3::MotionTpl represents a spatial motion (linear and angular velocities), e.g. a spatial velocity or acceleration associated to a frame or a body. The spatial motion is the mathematical representation of \f$ se(3) \f$.
- se3::ForceTpl represents a spatial force, e.g. a spatial impulse or force associated to a body. The spatial force is the mathematical representation of \f$ se^{*}(3) \f$, the dual of \f$ se(3) \f$.
- se3::InertiaTpl represents a spatial inertia characterizing a rigid body and expressed in a given frame. This inertia is composed of mass, the position of the center of mass regarding to the frame and a rotational inertia.

### Main Algorithms

### Collision checking and distance computation

Collision checking between two bodies of the kinematic chain and external
obstacles is implemented using library hpp-fcl (a modified version of
Flexible Collision Library). The computation of the position of objects in 3D
space for a given configuration is performed by the following classes:
- se3::GeometryModel that represents the collision objects associated to a joint stored in a se3::Model,
- se3::GeometryData that stores intermediate data like the position of objects in a given configuration of the kinematic chain.

## Further reading

This documentation starts with a preview of the [Mathematical Formulation of operations in
SE(3)](md_doc_a-maths_se3.html).

After this, we explain [how to use](md_doc_b-usage_intro.html) this library in different use cases, and we provide
[quick tutorials](md_doc_c-tutorials_intro.html) for classical tasks.

If you need an in-depth understanding of the topic, we also have [Labs](md_doc_d-labs_intro.html) for you.
