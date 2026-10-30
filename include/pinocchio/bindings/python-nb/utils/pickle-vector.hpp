// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// \brief Pickle visitor for std::vector (and aligned vector) types bound via nb::bind_vector.
///
/// \tparam Vector Vector type to pickle (e.g. std::vector<Vector3>).
template<class Vector>
struct PickleVectorVisitor : nb::def_visitor<PickleVectorVisitor<Vector>>
{
  using value_type = typename Vector::value_type;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl.def(
        "__getstate__",
        [](const Vector & self) {
          nb::list state;
          for (const auto & v : self)
            state.append(v);
          return state;
        })
      .def("__setstate__", [](Vector & self, nb::list state) {
        new (&self) Vector();
        for (auto item : state)
          self.push_back(nb::cast<value_type>(item));
      });
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
