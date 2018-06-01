# Creating models
<!--
//
// Copyright (c) 2016, 2018 CNRS
// Author: Florent Lamiraux, Justin Carpentier, Florian Valenza, Guilhem Saurel
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

Here you can find the differents ways to create a model ( simple models, python/lua/urdf loading, Joint after Joint)

## Introduction

In Pinocchio you can create Models in many ways. We have built-in parsers for different kind of format ( urdf, python, Lua)
but you can also create a Model from scratch and fill it with the Joints you want.

## Supported formats


### Format urdf

To load an urdf file in C++ code, copy the following lines:
```cpp
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/urdf.hpp>

bool verbose = false;
const std::string filename = "path/to/file/model.urdf";

se3::Model model
se3::urdf::buildModel (filename, model, verbose);
se3::Data data (model);
```

### Format Python

To load a python file in C++ code, copy the following lines:
```cpp
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/python.hpp"

bool verbose = false;
const std::string filename = "path/to/file/model.py";
const std::string model_name = "python_variable_name_in_filename"

se3::Model model = se3::python::buildModel(filename, model_name, verbose);
se3::Data data (model);
```

### Format lua

To load an lua file in C++ code, copy the following lines:
```cpp
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/lua.hpp>

bool verbose = false;
bool freeflyer = true;
const std::string filename = "path/to/file/model.lua";

se3::Model model = se3::lua::buildModel (filename, freeflyer, verbose);
se3::Data data (model);
```
