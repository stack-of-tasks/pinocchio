// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/utils/boost-variant.hpp"

#include "pinocchio/parsers/graph.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/tuple.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

void exposeModelGraphAlgo(nb::module_ m)
{
  using namespace pinocchio::graph;
  m.def(
    "buildModel", &pinocchio::graph::buildModel, "graph"_a, "root_body"_a, "root_position"_a,
    "root_joint"_a = JointFixed(), "root_joint_name"_a = "root_joint",
    "Build a pinocchio model based on the graph.");

  m.def(
    "buildModelWithBuildInfo",
    [](
      const ModelGraph & graph, const std::string & root_body, const SE3 & root_position,
      const JointVariant & root_joint, const std::string & root_joint_name) {
      auto ret = pinocchio::graph::buildModelWithBuildInfo(
        graph, root_body, root_position, root_joint, root_joint_name);
      return std::make_tuple(ret.model, ret.build_info);
    },
    "graph"_a, "root_body"_a, "root_position"_a, "root_joint"_a = JointFixed(),
    "root_joint_name"_a = "root_joint", "Build a pinocchio model based on the graph.");

  m.def(
    "merge", &pinocchio::graph::merge, "graph1"_a, "graph2"_a, "g1_body"_a, "g2_body"_a,
    "pose_g2_body_in_g1"_a, "merging_joint"_a = JointFixed(),
    "merging_joint_name"_a = "merging_joint",
    "Merge two ModelGraphs together by adding an edge between specified bodies.");

  nb::bind_vector<std::vector<Eigen::VectorXd>>(m, "StdVec_VectorXs");

  m.def(
    "lockJoints", &pinocchio::graph::lockJoints, "graph"_a, "joints_to_lock"_a,
    "reference_configurations"_a,
    "Lock specified joints in a ModelGraph at given reference configurations.");

  m.def(
    "prefixNames", &pinocchio::graph::prefixNames, "graph"_a, "prefix"_a,
    "Add a prefix to all names (body and joints) in the graph g. Useful to use before merging "
    "two model graphs.");
}

PINOCCHIO_PYTHON_NAMESPACE_END