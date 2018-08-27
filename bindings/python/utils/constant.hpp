//
// Copyright (c) 2018 CNRS
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

#ifndef __se3_python_utils_constant_hpp__
#define __se3_python_utils_constant_hpp__

#include <boost/python/scope.hpp>

namespace boost
{
  namespace python
  {
    
    ///
    /// \brief Define a constant given its value and a name within the current Boost Python scope.
    ///
    /// \tparam T Type of the constant.
    ///
    /// \param[in] name Name of the constant.
    /// \param[in] value Value of the constant.
    ///
    template<typename T>
    void def_constant(const char * name, const T & value)
    {
      namespace bp = boost::python;
      bp::scope().attr(name) = value;
    }
    
  } // namespace python
} // namespace boost

#endif // ifndef __se3_python_utils_constant_hpp__
