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

#ifndef __se3_python_utils_printable_hpp__
#define __se3_python_utils_printable_hpp__

#include <boost/python.hpp>

namespace se3
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    ///
    /// \brief Set the Python method __str__ and __repr__ to use the overloading operator<<.
    ///
    template<class C>
    struct PrintableVisitor : public bp::def_visitor< PrintableVisitor<C> >
    {
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
        .def(bp::self_ns::str(bp::self_ns::self))
        .def(bp::self_ns::repr(bp::self_ns::self))
        ;
      }
    };
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_utils_printable_hpp__
