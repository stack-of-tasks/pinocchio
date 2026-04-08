// Copyright (c) 2026 INRIA
#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/math.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// Implements the __array__ method on the binding for the parameter type C, enabling the following
/// in Python:
///
///     obj_arr = np.array(obj)  # copy=True implicit, returns a view
///     obj_arr = np.array(obj, copy=False)  # returns a copy
///
/// See \ref Force, \ref Motion for examples.
///
/// \remark This assumes that C has a toVector() method.
template<class C>
struct ToNumpyArrayVisitor : nb::def_visitor<ToNumpyArrayVisitor<C>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    using ToVectorConstReturnType = typename C::ToVectorConstReturnType;
    using ToVectorPlain = std::decay_t<ToVectorConstReturnType>;
    cl.def(
      "__array__",
      [](const C & self, nb::object /*dtype*/, nb::bool_ copy) {
        decltype(auto) v = self.toVector();
        if (bool(copy))
          return nb::cast(ToVectorPlain(v));
        else
          return nb::cast(make_ref(v));
      },
      nb::rv_policy::reference_internal, "dtype"_a = nb::none(), nb::kw_only(),
      "copy"_a = nb::none());
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
