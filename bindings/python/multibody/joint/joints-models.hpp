//
// Copyright (c) 2015 CNRS
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

#ifndef __se3_python_joints_models_hpp__
#define __se3_python_joints_models_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/joint/joint-collection.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;


    // generic expose_constructor : do nothing special
    template <class T>
    inline bp::class_<T>& expose_constructors(bp::class_<T>& cl)
    {
      return cl;
    }

    template<>
    inline bp::class_<JointModelRevoluteUnaligned>& expose_constructors<JointModelRevoluteUnaligned> (bp::class_<JointModelRevoluteUnaligned> & cl)
    {
      return cl.def(bp::init<double, double, double> ((bp::args("x"), bp::args("y"), bp::args("z")), "Init JointRevoluteUnaligned from the components x, y, z of the axis"));
    }
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_joint_models_hpp__
