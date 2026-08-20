// Copyright (c) 2026 INRIA

#pragma once

#include "pinocchio/bindings/python-nb/fwd.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

void exposeFramesGraph(nb::module_ m);
void exposeJointsGraph(nb::module_ m);
void exposeJointLimits(nb::module_ m);
void exposeEdgesAlgo(nb::module_ m);
void exposeGeometriesVariant(nb::module_ m);
void exposeGeometryGraph(nb::module_ m);
void exposeGeometryBuilder(nb::module_ m);
void exposeModelGraph(nb::module_ m);
void exposeModelGraphAlgo(nb::module_ m);
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS
void exposeAlgoGeometry(nb::module_ m);
#endif
void exposeModelConfigurationConverter(nb::module_ m);

inline void exposeGraph(nb::module_ m)
{
  nb::module_ m_graph = m.def_submodule("graph", "Graph-based incremental model construction.");

  exposeFramesGraph(m_graph);
  exposeJointsGraph(m_graph);
  exposeJointLimits(m_graph);
  exposeEdgesAlgo(m_graph);
  exposeGeometriesVariant(m_graph);
  exposeGeometryGraph(m_graph);
  exposeGeometryBuilder(m_graph);
  exposeModelGraph(m_graph);
  exposeModelGraphAlgo(m_graph);
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS
  exposeAlgoGeometry(m_graph);
#endif
  exposeModelConfigurationConverter(m_graph);
}

PINOCCHIO_PYTHON_NAMESPACE_END
