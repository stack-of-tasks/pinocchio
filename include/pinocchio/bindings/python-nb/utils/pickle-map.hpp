// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

/// \brief Pickle visitor for std::map (and aligned map) types bound via nb::bind_map.
///
/// \tparam Map Map type to pickle (e.g. std::map<std::string, VectorXs>).
template<class Map>
struct PickleMapVisitor : nb::def_visitor<PickleMapVisitor<Map>>
{
  using key_type = typename Map::key_type;
  using mapped_type = typename Map::mapped_type;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    cl.def(
        "__getstate__",
        [](const Map & self) {
          nb::dict state;
          for (const auto & kv : self)
            state[nb::cast(kv.first)] = kv.second;
          return state;
        })
      .def("__setstate__", [](Map & self, nb::dict state) {
        new (&self) Map();
        for (auto item : state)
          self[nb::cast<key_type>(item.first)] = nb::cast<mapped_type>(item.second);
      });
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
