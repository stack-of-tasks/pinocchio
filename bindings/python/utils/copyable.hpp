//
// Copyright (c) 2016-2023 CNRS INRIA, Heriot-Watt University
//

#ifndef __pinocchio_python_utils_copyable_hpp__
#define __pinocchio_python_utils_copyable_hpp__

#include <boost/python.hpp>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;

    template <class T>
    inline PyObject* managingPyObject(T* p)
    {
      return typename bp::manage_new_object::apply<T*>::type()(p);
    }

    template <class Copyable>
    bp::object generic__copy__(bp::object copyable)
    {
      Copyable* newCopyable(new Copyable(bp::extract<const Copyable&>(copyable)));
      bp::object result(bp::detail::new_reference(managingPyObject(newCopyable)));

      bp::extract<bp::dict>(result.attr("__dict__"))().update(copyable.attr("__dict__"));

      return result;
    }

    template <class Copyable>
    bp::object generic__deepcopy__(bp::object copyable, bp::dict memo)
    {
      bp::object copyMod = bp::import("copy");
      bp::object deepcopy = copyMod.attr("deepcopy");

      Copyable* newCopyable(new Copyable(bp::extract<const Copyable&>(copyable)));
      bp::object result(bp::detail::new_reference(managingPyObject(newCopyable)));

      int copyableId = (long long)(copyable.ptr());
      memo[copyableId] = result;

      bp::extract<bp::dict>(result.attr("__dict__"))().update(
          deepcopy(bp::extract<bp::dict>(copyable.attr("__dict__"))(), memo));
      return result;
    }


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
        cl.def("__copy__", &generic__copy__<C>);
        cl.def("__deepcopy__", &generic__deepcopy__<C>);
      }
      
    private:
      static C copy(const C & self) { return C(self); }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_copyable_hpp__
