//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_multibody_model_graph_hpp__
#define __pinocchio_multibody_model_graph_hpp__

#include "pinocchio/config.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"

#include "pinocchio/parsers/graph/joints.hpp"

#include <Eigen/Core>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/visitors.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>

namespace pinocchio
{

  /// @brief Represents a vertex (body) in the model graph.
  ///
  /// A vertex corresponds to a rigid body in the multibody model.
  /// For now, a vertex represents only a body, but this may evolve to include
  /// sensors, actuators, or other entities in the future.
  struct ModelGraphVertex
  {
    /// @brief Unique name of the body.
    std::string name;

    /// @brief Spatial inertia of the body, expressed at its center of mass (CoM).
    ///
    /// Note: If the joint is reversed in the model graph, the body frame pose
    /// is kept the same in the model, so this inertia remains valid.
    Inertia inertia;
  };

  /// @brief Represents an edge (joint) in the model graph.
  struct ModelGraphEdge
  {
    /// @brief Unique name of the joint
    std::string name;

    /// @brief What is the type of the joint
    JointGraphVariant joint;

    /// @brief Transformation from the previous vertex to edge
    ///
    /// Correspond to the transformation from body supporting joint to said joint
    SE3 out_to_joint;
    /// @brief Transformation from edge to next vertex
    ///
    /// Correspond to the transformation from the current joint to its supported body.
    SE3 joint_to_in;

    /// @brief boolean to know if we are in a forward or backward edge
    bool reverse = false;
  };

  /// @brief Represents multibody model as a bidirectional graph.
  ///
  /// This is an intermediate step before creating a model, that
  /// allows more flexibility as to which body will be the root...
  struct PINOCCHIO_DLLAPI ModelGraph
  {
    typedef boost::
      adjacency_list<boost::vecS, boost::vecS, boost::directedS, ModelGraphVertex, ModelGraphEdge>
        Graph;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexDesc;
    typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

    /// \brief Add a new vertex (body) to the graph
    ///
    /// \param[in] vertex_name Name of the vertex
    /// \param[in] inertia pinocchio inertia of the body taken at the Center of Mass of said body.
    void addBody(const std::string & vertex_name, const Inertia & inertia);

    /// \brief Add edges (joint) to the graph. Since it's a bidirectional graph,
    /// edge and its reverse are added to the graph.
    ///
    /// \param[in] joint_name Name of the edge
    /// \param[in] joint Type of the joint
    /// \param[in] out_body Vertex that is supporting the edge
    /// \param[in] out_to_joint Transformation from supporting vertex to edge
    /// \param[in] in_body Vertex that is supported by edge
    /// \param[in] joint_to_in Transformation from edge to supported vertex
    ///
    /// \note Since it's a bidirectional graph, two edges are added to the graph.
    /// Joints and transformation are inverted, to create reverse edge.
    void addJoint(
      const std::string & joint_name,
      const JointGraphVariant & joint,
      const std::string & out_body,
      const SE3 & out_to_joint,
      const std::string & in_body,
      const SE3 & joint_to_in);

    /// @brief  Build a pinocchio model based on the graph that was built previously, that allows to
    /// have a root_joint.
    ///
    /// @param root_body First body to add to the model
    /// @param root_position position of said body wrt to the universe
    /// @param root_joint joint that will append to the root_body
    /// @param root_joint_name name of the first joint in the model
    ///
    /// @return a pinocchio model
    Model buildModel(
      const std::string & root_body,
      const pinocchio::SE3 & root_position,
      const boost::optional<JointGraphVariant> & root_joint = boost::none,
      const std::string & root_joint_name = "root_joint") const;

    void copyGraph(const ModelGraph & g, const std::string & prefix = "");

    /// @brief Boost graph structure that holds the graph structure
    Graph g;
    /// @brief Name of the vertexes in the graph. Useful for graph parcours.
    std::unordered_map<std::string, VertexDesc> name_to_vertex;
  };

  PINOCCHIO_DLLAPI ModelGraph mergeGraphs(
    const ModelGraph & g1,
    const ModelGraph & g2,
    const std::string & g1_body,
    const std::string & g2_body,
    const SE3 & pose_g2_body_in_g1,
    const boost::optional<JointGraphVariant> & merging_joint = boost::none,
    const std::string & merging_joint_name = "merging_joint",
    const std::string & g2_prefix = "g2/");
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_graph_hpp__
