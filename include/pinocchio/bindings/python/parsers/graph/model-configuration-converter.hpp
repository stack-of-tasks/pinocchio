//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_python_parsers_graph_model_configuration_converter_hpp__
#define __pinocchio_python_parsers_graph_model_configuration_converter_hpp__

#include "pinocchio/parsers/graph/model-configuration-converter.hpp"

namespace pinocchio
{
  namespace graph
  {
    namespace python
    {
      namespace bp = boost::python;

      template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
      struct ModelConfigurationConverterVisitor
      : public boost::python::def_visitor<
          ModelConfigurationConverterVisitor<_Scalar, _Options, JointCollectionTpl>>
      {
        typedef _Scalar Scalar;
        enum
        {
          Options = _Options
        };
        typedef JointCollectionTpl<Scalar, Options> JointCollection;
        typedef ModelConfigurationConverterTpl<Scalar, Options, JointCollectionTpl>
          ModelConfigurationConverter;

        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;

        static VectorXs convert_configuration_vector_proxy(
          const ModelConfigurationConverter & converter, const VectorXs & q_source)
        {
          VectorXs q_target(VectorXs::Zero(converter._target_configuration_size));
          converter.convertConfigurationVector(q_source, q_target);
          return q_target;
        }

        static VectorXs convert_tangent_vector_proxy(
          const ModelConfigurationConverter & converter,
          const VectorXs & q_source,
          const VectorXs & v_source)
        {
          VectorXs v_target(VectorXs::Zero(converter._target_tangent_size));
          converter.convertTangentVector(q_source, v_source, v_target);
          return v_target;
        }

        template<class PyClass>
        void visit(PyClass & cl) const
        {
          cl.def(
            "convertConfigurationVector",
            &ModelConfigurationConverter::template convertConfigurationVector<VectorXs, VectorXs>,
            bp::args("self", "q_source", "q_target"),
            "Convert q_source configuration vector from source model to q_target configuration "
            "vector from target model.");
          cl.def(
            "convertTangentVector",
            &ModelConfigurationConverter::template convertTangentVector<
              VectorXs, VectorXs, VectorXs>,
            bp::args("self", "q_source", "q_target"),
            "Convert q_source configuration vector from source model to q_target configuration "
            "vector from target model.");
          cl.def(
            "convertConfigurationVector",
            &ModelConfigurationConverterVisitor::convert_configuration_vector_proxy,
            bp::args("self", "q_source"),
            "Return q_source configuration vector converted to target model.");
          cl.def(
            "convertTangentVector",
            &ModelConfigurationConverterVisitor::convert_tangent_vector_proxy,
            bp::args("self", "q_source", "v_source"),
            "Return v_source configuration vector converted to target model.");
        }

        static void expose()
        {
          bp::class_<ModelConfigurationConverter>(
            "ModelConfigurationConverter",
            "Convert configuration or tangent vector from two model with different root.",
            bp::no_init)
            .def(ModelConfigurationConverterVisitor());

          bp::def("createConverter", createConverter<Scalar, Options, JointCollectionTpl>);
        }
      };

    } // namespace python
  } // namespace graph
} // namespace pinocchio

#endif // define __pinocchio_python_parsers_graph_model_configuration_converter_hpp__
