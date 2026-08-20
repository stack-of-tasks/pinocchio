// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/boost-variant.hpp"

#include "pinocchio/parsers/graph.hpp"

#include <nanobind/stl/string.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeModelGraph(nb::module_ m)
{
  using namespace pinocchio::graph;

  nb::class_<ModelGraphBuildInfo>(
    m, "ModelGraphBuildInfo",
    "Contains information about how buildModel walked the ModelGraph to construct a Model")
    .def(nb::init<>(), "Default constructor.");

  nb::class_<ModelGraph>(m, "ModelGraph", "Represents multibody model as a bidirectional graph.")
    .def(nb::init<>(), "Default constructor.")
    .def(
      "addFrame", &ModelGraph::addFrame, "vertex_name"_a, "frame"_a,
      "Add a new vertex to the graph.")
    .def(
      "addBody", &ModelGraph::addBody, "vertex_name"_a, "inertia"_a,
      "Add a new body (vertex with inertia) to the graph.")
    .def(
      "addGeometry", &ModelGraph::addGeometry, "body_name"_a, "geometry"_a,
      "Add a geometry to the vertex associated with body_name")
    .def(
      "addGeometries", &ModelGraph::addGeometries, "body_name"_a, "geometries"_a,
      "Add a vector of geometry to the vertex associated with body_name")
    .def(
      "geometryBuilder", &ModelGraph::geometryBuilder,
      "Return a GeometryBuilder to add Geometries to vertices")
    .def(
      "addJoint",
      static_cast<void (ModelGraph::*)(
        const std::string &, const JointVariant &, const std::string &, const SE3 &,
        const std::string &, const SE3 &)>(&ModelGraph::addJoint),
      "joint_name"_a, "joint"_a, "source_body"_a, "source_to_joint"_a, "target_body"_a,
      "joint_to_target"_a,
      "Add edges (joint) to the graph. Since it's a bidirectional graph,\n"
      "edge and its reverse are added to the graph.\n")
    .def(
      "addJoint", (void (ModelGraph::*)(const EdgeParameters &))&ModelGraph::addJoint, "params"_a,
      "Add edges (joint) to the graph using EdgeParameters.")
    .def(
      "edgeBuilder", &ModelGraph::edgeBuilder, "Returns an EdgeBuilder object to construct edges.")
    .def(
      "appendGraph", &ModelGraph::appendGraph, "g"_a,
      "Copies another ModelGraph into this one. Use with caution, because no edges are created "
      "to connect the new graph to itself.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
