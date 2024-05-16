//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_utils_comparable_hpp__
#define __pinocchio_python_utils_comparable_hpp__

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    ///
    /// \brief Add the Python method == and != to allow a comparison of this.
    ///
    template<class C, bool has_comparison_operators = true>
    struct ComparableVisitor
    : public bp::def_visitor<ComparableVisitor<C, has_comparison_operators>>
    {
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(bp::self == bp::self).def(bp::self != bp::self);
      }
    };

    template<class C>
    struct ComparableVisitor<C, false> : public bp::def_visitor<ComparableVisitor<C, false>>
    {
      template<class PyClass>
      void visit(PyClass &) const
      {
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_comparable_hpp__
