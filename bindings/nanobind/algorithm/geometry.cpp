// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/geometry.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeGeometryAlgo(nb::module_ m)
{
  m.def(
    "updateGeometryPlacements",
    [](
      const Model & model, Data & data, const GeometryModel & geom_model, GeometryData & geom_data,
      ConstVectorRef q) { updateGeometryPlacements(model, data, geom_model, geom_data, q); },
    "model"_a, "data"_a, "geometry_model"_a, "geometry_data"_a, "q"_a,
    "Update the placement of the collision objects according to the current configuration.\n"
    "The algorithm also updates the current placement of the joints in data.");

  m.def(
    "updateGeometryPlacements",
    [](
      const Model & model, const Data & data, const GeometryModel & geom_model,
      GeometryData & geom_data) { updateGeometryPlacements(model, data, geom_model, geom_data); },
    "model"_a, "data"_a, "geometry_model"_a, "geometry_data"_a,
    "Update the placement of the collision objects according to the current joint placements "
    "stored in data.");
}
PINOCCHIO_PYTHON_NAMESPACE_END;
