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

/** \page pinocchio_page_loading Loading a model

\section pinocchio_page_loading_introduction Introduction

The most convenient way to build a robot model consists in parsing a description
file.

\section pinocchio_page_loading_supported_formats Supported formats

Two format are supported.

\subsection pinocchio_page_loading_urdf Format urdf

To load an urdf file in C++ code, copy the following lines:
\code
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/urdf.hpp>
#include <pinocchio/multibody/parser/utils.hpp>
bool verbose = false;
se3::Model model = se3::urdf::buildModel (filename, verbose);
\endcode

\subsection pinocchio_page_loading_lua Format lua

To load an lua file in C++ code, copy the following lines:
\code
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/lua.hpp>
#include <pinocchio/multibody/parser/utils.hpp>
bool verbose = false;
se3::Model model = se3::lua::buildModel (filename, false, verbose);
\endcode

 */
