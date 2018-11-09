//
// Copyright (c) 2016 CNRS
//

#ifndef __pinocchio_python_utils_printable_hpp__
#define __pinocchio_python_utils_printable_hpp__

#include <boost/python.hpp>

namespace pinocchio
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
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_printable_hpp__
