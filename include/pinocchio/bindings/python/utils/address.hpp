//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_python_utils_address_hpp__
#define __pinocchio_python_utils_address_hpp__

#include <eigenpy/eigenpy.hpp>

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    ///
    /// \brief Add the Python method to extract the address of the underlying C++ object.
    ///
    template<class C>
    struct AddressVisitor : public bp::def_visitor<AddressVisitor<C>>
    {

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl.def(
          "__address__",
          +[](const C & self) -> size_t {
            return reinterpret_cast<size_t>(static_cast<const void *>(&self));
          },
          bp::arg("self"), "Returns the address of the underlying C++ object.");
      }
    };

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_address_hpp__
