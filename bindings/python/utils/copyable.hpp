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

#ifndef __pinocchio_python_utils_copyable_hpp__
#define __pinocchio_python_utils_copyable_hpp__

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    ///
    /// \brief Add the Python method copy to allow a copy of this by calling the copy constructor.
    ///
    template<class C>
    struct CopyableVisitor : public bp::def_visitor< CopyableVisitor<C> >
    {
      template<class PyClass>
      void visit(PyClass & cl) const
      { cl.def("copy",&copy,"Returns a copy of *this."); }
      
    private:
      static C copy(const C & self) { return C(self); }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_copyable_hpp__
