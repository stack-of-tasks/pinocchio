// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/parsers/graph.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
struct ModelConfigurationConverterVisitor
: public nb::def_visitor<ModelConfigurationConverterVisitor<_Scalar, _Options, JointCollectionTpl>>
{
  typedef _Scalar Scalar;
  static constexpr int Options = _Options;
  typedef JointCollectionTpl<Scalar, Options> JointCollection;
  typedef pinocchio::graph::ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
    ModelConfigurationConverter;

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;

  template<class PyClass, class... Extra>
  void execute(PyClass & cl, const Extra &...) const
  {
    using namespace nb::literals;
    cl.def(
        "convertConfigurationVector",
        [](
          const ModelConfigurationConverter & converter, Eigen::Ref<const VectorXs> q_source,
          Eigen::Ref<VectorXs> q_target) {
          converter.convertConfigurationVector(q_source, q_target);
        },
        "q_source"_a, "q_target"_a,
        "Convert q_source configuration vector from source model to q_target configuration "
        "vector from target model.")
      .def(
        "convertTangentVector",
        [](
          const ModelConfigurationConverter & converter, Eigen::Ref<const VectorXs> q_source,
          Eigen::Ref<const VectorXs> v_source, Eigen::Ref<VectorXs> v_target) {
          converter.convertTangentVector(q_source, v_source, v_target);
        },
        "q_source"_a, "v_source"_a, "v_target"_a,
        "Convert q_source configuration vector and v_source tangent vector from source model to "
        "v_target tangent vector from target model.")
      .def(
        "convertConfigurationVector",
        [](const ModelConfigurationConverter & converter, const VectorXs & q_source) {
          VectorXs q_target(VectorXs::Zero(converter._target_configuration_size));
          converter.convertConfigurationVector(q_source, q_target);
          return q_target;
        },
        "q_source"_a, "Return q_source configuration vector converted to target model.")
      .def(
        "convertTangentVector",
        [](
          const ModelConfigurationConverter & converter, const VectorXs & q_source,
          const VectorXs & v_source) {
          VectorXs v_target(VectorXs::Zero(converter._target_tangent_size));
          converter.convertTangentVector(q_source, v_source, v_target);
          return v_target;
        },
        "q_source"_a, "v_source"_a, "Return v_source tangent vector converted to target model.");
  }
};

PINOCCHIO_PYTHON_NAMESPACE_END
