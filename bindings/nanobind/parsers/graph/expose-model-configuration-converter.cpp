//
// Copyright (c) 2026 INRIA
//

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/parsers/graph/model-configuration-converter.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeModelConfigurationConverter(nb::module_ m)
{
  using namespace pinocchio::graph;
  using ModelConfigurationConverter = pinocchio::graph::ModelConfigurationConverterTpl<
    Scalar, Options, pinocchio::JointCollectionDefaultTpl>;

  nb::class_<ModelConfigurationConverter>(
    m, "ModelConfigurationConverter",
    "Convert configuration or tangent vector from two model with different root.")
    .def(
      ModelConfigurationConverterVisitor<Scalar, Options, pinocchio::JointCollectionDefaultTpl>());

  m.def(
    "createConverter",
    &pinocchio::graph::createConverter<Scalar, Options, pinocchio::JointCollectionDefaultTpl>,
    "model_source"_a, "model_target"_a, "build_info_source"_a, "build_info_target"_a);
}
PINOCCHIO_PYTHON_NAMESPACE_END
