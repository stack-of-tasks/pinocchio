//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_python_utils_std_vector_hpp__
#define __se3_python_utils_std_vector_hpp__

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <string>
#include <vector>

namespace se3
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    ///
    /// \brief Expose an std::vector from a type given as template argument.
    ///
    /// \tparam T Type to expose as std::vector<T>.
    ///
    /// \sa StdAlignedVectorPythonVisitor
    ///
    template<class T, bool NoProxy = false>
    struct StdVectorPythonVisitor
    : public bp::vector_indexing_suite< typename std::vector<T>,NoProxy >
    {
      
      static void expose(const std::string & class_name,
                         const std::string & doc_string = "")
      {
        bp::class_< std::vector<T> >(class_name.c_str(),doc_string.c_str())
        .def(StdVectorPythonVisitor());
      }
    };
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_utils_std_vector_hpp__
