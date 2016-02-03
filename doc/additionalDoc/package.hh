//
// Copyright (c) 2016 CNRS
// Author: Florent Lamiraux
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

/** \mainpage
\section pinocchio_section_introduction Introduction

This package implements efficient forward kinematics algorithms for multi-body
kinematic chains. One of the main features is the separation between models (constant values representing an object) and data (used for intermediate computations).

The main classes are
\li se3::Model that represents a kinematic chain composed of joints that move
    links with mass and inertia,
\li se3::Data that stores intermediate data for kinematic computations.

\section pinocchio_section_kinematics Kinematics

The library provides some classes to represent kinematic objects:
\li se3::SE3Tpl represents a rigid-body transformation.

\section pinocchio_section_collision_checking Collision Checking

Collision checking between bodies of the kinematic chain and external
obstacles is implemented using library hpp-fcl (a modified version of
Flexible Collision Library). The computation of the position of objects in 3D
space for a given configuration is performed by the following classes:
\li se3::GeometryModel that represents a kinematic chain moving objects,
\li se3::GeometryData that stores intermediate data like the position of objects
in a given configuration of the kinematic chain.
**/
