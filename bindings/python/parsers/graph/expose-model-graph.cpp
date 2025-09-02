//
// Copyright (c) 2025 INRIA
//

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>

#include "pinocchio/bindings/python/parsers/model-graph.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeModelGraph()
    {
      using namespace pinocchio::graph;

      bp::class_<ModelGraphBuildInfo> model_grap_build_info(
        "ModelGraphBuildInfo",
        "Contains information about how buildModel walked the ModelGraph to construct a Model");

      bp::class_<ModelGraph>(
        "ModelGraph", "Represents multibody model as a bidirectional graph.", bp::init<>())
        .def(
          "addFrame", &ModelGraph::addFrame,
          (bp::arg("self"), bp::arg("vertex_name"), bp::arg("frame")),
          "Add a new vertex to the graph.")
        .def(
          "addBody", &ModelGraph::addBody,
          (bp::arg("self"), bp::arg("vertex_name"), bp::arg("inertia")),
          "Add a new body (vertex with inertia) to the graph.")
        .def(
          "addGeometry", &ModelGraph::addGeometry, (bp::arg("body_name"), bp::arg("geometry")),
          "Add a geometry to the vertex associated with body_name")
        .def(
          "addGeometries", &ModelGraph::addGeometry, (bp::arg("body_name"), bp::arg("geometries")),
          "Add a vector of geometry to the vertex associated with body_name")
        .def(
          "geometryBuilder", &ModelGraph::geometryBuilder, bp::arg("self"),
          "Return a GeometryBuilder to add Geometries to vertices")
        .def(
          "addJoint",
          (void(ModelGraph::*)(
            const std::string &, const JointVariant &, const std::string &, const SE3 &,
            const std::string &, const SE3 &))
            & ModelGraph::addJoint,
          (bp::arg("self"), bp::arg("joint_name"), bp::arg("joint"), bp::arg("source_body"),
           bp::arg("source_to_joint"), bp::arg("target_body"), bp::arg("joint_to_target")),
          "Add edges (joint) to the graph. Since it's a bidirectional graph,\n"
          "edge and its reverse are added to the graph.\n")
        .def(
          "addJoint", (void(ModelGraph::*)(const EdgeParameters &)) & ModelGraph::addJoint,
          (bp::arg("self"), bp::arg("params")),
          "Add edges (joint) to the graph using EdgeParameters.")
        .def(
          "edgeBuilder", &ModelGraph::edgeBuilder, bp::arg("self"),
          "Returns an EdgeBuilder object to construct edges.")
        .def(
          "appendGraph", &ModelGraph::appendGraph, (bp::arg("self"), bp::arg("g")),
          "Copies another ModelGraph into this one. Use with caution, because no edges are created "
          "to connect the new graph to itself.");
    }
  } // namespace python
} // namespace pinocchio
