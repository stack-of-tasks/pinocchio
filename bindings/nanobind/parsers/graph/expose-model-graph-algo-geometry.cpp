// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/parsers/graph.hpp"
#include "../conversion-util.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeAlgoGeometry(nb::module_ m)
{
  using namespace pinocchio::graph;

  m.def(
    "buildGeometryModel",
    [](const ModelGraph & g, const Model & model, const GeometryType type, nb::object mesh_loader) {
      return buildGeometryModel(g, model, type, meshLoaderFromObject(std::move(mesh_loader)));
    },
    "g"_a, "model"_a, "type"_a, "mesh_loader"_a = nb::none(),
    "Build a pinocchio model based on the graph.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
