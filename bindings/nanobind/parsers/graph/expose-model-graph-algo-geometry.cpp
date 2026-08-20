// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/parsers/graph.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeAlgoGeometry(nb::module_ m)
{
  using namespace pinocchio::graph;

  m.def(
    "buildGeometryModel", &buildGeometryModel, "g"_a, "model"_a, "type"_a,
    "mesh_loader"_a = ::coal::MeshLoaderPtr(), "Build a pinocchio model based on the graph.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
