//
// Copyright (c) 2016-2023 CNRS INRIA
//

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
    struct CopyableVisitor
    : public bp::def_visitor< CopyableVisitor<C> >
    {
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def("copy",&copy,bp::arg("self"),"Returns a copy of *this.");
        cl.def("__copy__", &copy,bp::arg("self"),"Returns a copy of *this.");
        cl.def("__deepcopy__", &deepcopy,bp::args("self","memo"),"Returns a deep copy of *this.");
      }
      
    private:
      static C copy(const C & self) { return C(self); }
      static C deepcopy(const C & self, bp::dict) { return C(self); }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_copyable_hpp__
