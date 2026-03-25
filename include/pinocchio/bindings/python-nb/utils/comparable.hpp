// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// \brief Set the Python methods __eq__ and __ne__ to use the overloaded operators == and !=.
template<class C>
struct ComparableVisitor : public nb::def_visitor<ComparableVisitor<C>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl.def("__eq__", [](const C & a, const C & b) { return a == b; }, nb::is_operator());
    cl.def("__ne__", [](const C & a, const C & b) { return a != b; }, nb::is_operator());
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
