// Copyright (c) 2026 INRIA

#pragma once

#include <sstream>

#include "../fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// \brief Set the Python methods __str__ and __repr__ to use the overloading operator<<.
template<class C>
struct PrintableVisitor : public nb::def_visitor<PrintableVisitor<C>>
{
  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl.def("__str__", [](const C & self) {
      std::ostringstream oss;
      oss << self;
      return oss.str();
    });
    cl.def("__repr__", [](const C & self) {
      std::ostringstream oss;
      oss << self;
      return oss.str();
    });
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
