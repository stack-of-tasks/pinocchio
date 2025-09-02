//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_python_parsers_model_graph_hpp__
#define __pinocchio_python_parsers_model_graph_hpp__

#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>

#include "pinocchio/bindings/python/fwd.hpp"

namespace pinocchio
{
  namespace python
  {
    template<typename Scalar>
    struct GraphNamespace
    {
    };

    void exposeFramesGraph();
    void exposeJointsGraph();
    void exposeJointLimits();
    void exposeEdgesAlgo();
    void exposeGeometriesVariant();
    void exposeGeometryGraph();
    void exposeGeometryBuilder();
    void exposeModelGraph();
    void exposeModelGraphAlgo();

    void exposeAlgoGeometry();

    void exposeModelConfigurationConverter();

    inline void exposeGraph()
    {
      boost::python::scope graph = boost::python::class_<GraphNamespace<context::Scalar>>("graph");

#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
      exposeFramesGraph();
      exposeJointsGraph();
      exposeJointLimits();
      exposeEdgesAlgo();
      exposeGeometriesVariant();
      exposeGeometryGraph();
      exposeGeometryBuilder();
      exposeModelGraph();
      exposeModelGraphAlgo();
  #if defined(PINOCCHIO_WITH_HPP_FCL)
      exposeAlgoGeometry();
  #endif // if defined(PINOCCHIO_WITH_HPP_FCL)
#endif   // if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
      exposeModelConfigurationConverter();
    }

  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_parsers_mjcf_hpp__
