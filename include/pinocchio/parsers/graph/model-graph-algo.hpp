//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_model_graph_algo_hpp__
#define __pinocchio_parsers_graph_model_graph_algo_hpp__

#include "pinocchio/parsers/graph/fwd.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/joints.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace pinocchio
{
  namespace graph
  {
    /// @brief \ref buildModelWithBuildInfo return value.
    struct BuildModelWithBuildInfoReturn
    {
      /// Generated model.
      Model model;
      ModelGraphBuildInfo build_info;
    };

    /// @brief  Build a pinocchio model based on the graph that was built previously, that allows
    /// to have a root_joint.
    ///
    /// @param root_body First body to add to the model
    /// @param root_position position of said body wrt to the universe
    /// @param root_joint joint that will append to the root_body. by default, it will be fixed
    /// @param root_joint_name name of the first joint in the model
    ///
    /// @return A pinocchio model
    PINOCCHIO_PARSERS_DLLAPI Model buildModel(
      const ModelGraph & g,
      const std::string & root_body,
      const SE3 & root_position,
      const JointVariant & root_joint = JointFixed(),
      const std::string & root_joint_name = "root_joint");

    /// @brief  Build a pinocchio model based on the graph that was built previously, that allows
    /// to have a root_joint.
    ///
    /// @param root_body First body to add to the model
    /// @param root_position position of said body wrt to the universe
    /// @param root_joint joint that will append to the root_body. by default, it will be fixed
    /// @param root_joint_name name of the first joint in the model
    ///
    /// @return A structure with a pinocchio \ref Model and information about how it has been
    /// built.
    PINOCCHIO_PARSERS_DLLAPI BuildModelWithBuildInfoReturn buildModelWithBuildInfo(
      const ModelGraph & g,
      const std::string & root_body,
      const SE3 & root_position,
      const JointVariant & root_joint = JointFixed(),
      const std::string & root_joint_name = "root_joint");

    /// @brief  Add a prefix to all bodies and joints in graph g. Mandatory step before using
    /// mergeGraphs.
    ///
    /// @param g graph
    /// @param prefix prefix to add to all names
    ///
    /// @return a model graph
    PINOCCHIO_PARSERS_DLLAPI ModelGraph
    prefixNames(const ModelGraph & g, const std::string & prefix);

    /// @brief  Merge 2 graphs together, by adding an edge between the two bodies in arguments.
    ///
    /// @param g1 First graph
    /// @param g2 Second graph
    /// @param g1_body body in g1 that will connect to g2
    /// @param g2_body body in g2 that will connect to g1
    /// @param pose_g2_body_in_g1 pose of g2_body wrt g1_body
    /// @param merging_joint joint that will connect the two bodies. By default, it will be a fixed
    /// joint.
    /// @param merging_joint_name name of the joint that will connect the two graphs
    ///
    /// @return a model graph
    PINOCCHIO_PARSERS_DLLAPI ModelGraph merge(
      const ModelGraph & g1,
      const ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      const JointVariant & merging_joint = JointFixed(),
      const std::string & merging_joint_name = "merging_joint");

    /// @brief  Transform a list of joints into its fixed version, locked into a reference
    /// configuratioh.
    ///
    /// @param g graph
    /// @param joints_to_lock List of joints to transform by name
    /// @param reference_configurations Configurations the joints will be locked into
    ///
    /// @return a model graph
    PINOCCHIO_PARSERS_DLLAPI ModelGraph lockJoints(
      const ModelGraph & g,
      const std::vector<std::string> & joints_to_lock,
      const std::vector<Eigen::VectorXd> & reference_configurations);

  } // namespace graph
} // namespace pinocchio
#endif // ifndef __pinocchio_parsers_graph_model_graph_algo_hpp__
