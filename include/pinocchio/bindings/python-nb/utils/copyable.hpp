// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<class C>
struct CopyableVisitor : nb::def_visitor<CopyableVisitor<C>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    const auto copy = [](const C & self) -> C { return C(self); };
    cl.def("copy", copy, "Returns a copy of this object.")
      .def("__copy__", copy, "Returns a copy of this object.")
      .def("__deepcopy__", [](const C & self, nb::dict) -> C { return C(self); });
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
