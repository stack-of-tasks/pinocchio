//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_multibody_model_graph_hpp__
#define __pinocchio_multibody_model_graph_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"

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
  struct JointFixedGraph
  {
    JointFixedGraph() = default;
  };

  struct JointRevoluteGraph
  {
    // rotation axis
    Eigen::Vector3d axis;

    JointRevoluteGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointRevoluteUnboundedGraph
  {
    Eigen::Vector3d axis;

    JointRevoluteUnboundedGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointPrismaticGraph
  {
    Eigen::Vector3d axis;
    JointPrismaticGraph(const Eigen::Vector3d & ax)
    : axis(ax)
    {
    }
  };

  struct JointFreeFlyerGraph
  {
    JointFreeFlyerGraph() = default;
  };

  struct JointSphericalGraph
  {
    JointSphericalGraph() = default;
  };

  // Flipped whne model is reversed ?
  struct JointSphericalZYXGraph
  {
    JointSphericalZYXGraph() = default;
  };

  struct JointTranslationGraph
  {
    JointTranslationGraph() = default;
  };

  struct JointPlanarGraph
  {
    JointPlanarGraph() = default;
  };

  struct JointHelicalGraph
  {
    Eigen::Vector3d axis;
    double pitch;

    JointHelicalGraph(const Eigen::Vector3d & ax, const double & p)
    : axis(ax)
    , pitch(p) {};
  };

  struct JointUniversalGraph
  {
    Eigen::Vector3d axis1;
    Eigen::Vector3d axis2;

    JointUniversalGraph(const Eigen::Vector3d & ax1, const Eigen::Vector3d & ax2)
    : axis1(ax1)
    , axis2(ax2) {};
  };

  // Mimic : joint name, ratio et offset

  // Forward declare
  struct JointCompositeGraph;
  // Forward declare
  struct JointMimicGraph;

  using JointGraphVariant = boost::variant<
    JointFixedGraph,
    JointRevoluteGraph,
    JointRevoluteUnboundedGraph,
    JointPrismaticGraph,
    JointFreeFlyerGraph,
    JointSphericalGraph,
    JointSphericalZYXGraph,
    JointTranslationGraph,
    JointPlanarGraph,
    JointHelicalGraph,
    JointUniversalGraph,
    boost::recursive_wrapper<JointCompositeGraph>,
    boost::recursive_wrapper<JointMimicGraph>>;

  struct JointCompositeGraph
  {
    std::vector<JointGraphVariant> joints;
    std::vector<SE3> jointsPlacements;

    JointCompositeGraph() = default;

    JointCompositeGraph(const JointGraphVariant & j, const SE3 & jPose)
    {
      joints.push_back(j);
      jointsPlacements.push_back(jPose);
    }

    JointCompositeGraph(const std::vector<JointGraphVariant> & js, const std::vector<SE3> & jPoses)
    : joints(js)
    , jointsPlacements(jPoses) {};

    void addJoint(const JointGraphVariant & jm, const SE3 & pose = SE3::Identity())
    {
      joints.push_back(jm);
      jointsPlacements.push_back(pose);
    }
  };

  struct JointMimicGraph
  {
    std::string primary_name;

    JointGraphVariant secondary_joint;
    double scaling;
    double offset;

    JointMimicGraph() = default;

    JointMimicGraph(
      const JointGraphVariant & jmodel_secondary,
      const std::string & name_primary,
      const double & scaling_,
      const double & offset_)
    : secondary_joint(jmodel_secondary)
    , primary_name(name_primary)
    , scaling(scaling_)
    , offset(offset_)
    {
    }
  };

  struct ReverseJointVisitor : public boost::static_visitor<JointGraphVariant>
  {
    typedef JointGraphVariant ReturnType;

    ReturnType operator()(const JointRevoluteGraph & joint) const
    {
      return joint;
    }

    ReturnType operator()(const JointRevoluteUnboundedGraph & joint) const
    {
      return joint;
    }

    ReturnType operator()(const JointPrismaticGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointFixedGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointFreeFlyerGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointSphericalGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointSphericalZYXGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointTranslationGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointPlanarGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointHelicalGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointUniversalGraph & joint) const
    {
      return JointUniversalGraph(-joint.axis2, -joint.axis1);
    }
    ReturnType operator()(const JointMimicGraph & joint) const
    {
      return joint;
    }
    ReturnType operator()(const JointCompositeGraph & joint) const
    {
      JointCompositeGraph jReturn;
      // Reverse joints
      for (size_t i = joint.joints.size(); i-- > 0;)
      {
        jReturn.joints.push_back(boost::apply_visitor(*this, joint.joints[i]));
        jReturn.jointsPlacements.push_back(joint.jointsPlacements[i].inverse());
      }
      return jReturn;
    }
  };

  template<typename Graph>
  struct RecordTreeEdgeVisitor : public boost::default_dfs_visitor
  {
    typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

    RecordTreeEdgeVisitor(std::vector<EdgeDesc> * edges)
    : edges(edges)
    {
    }

    void tree_edge(EdgeDesc edge_desc, const Graph &) const
    {
      edges->push_back(edge_desc);
    }

    std::vector<EdgeDesc> * edges;
  };

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

  // Add const to every operator (otherwise call fails)
  struct CreateJointModel : public boost::static_visitor<JointModel>
  {
    typedef JointModel ReturnType;

    ReturnType operator()(const JointFixedGraph & joint) const
    {
      PINOCCHIO_THROW_PRETTY(
        std::invalid_argument,
        "Graph - cannot create a fixed joint. In pinocchio, fixed joints are frame.");
    }
    ReturnType operator()(const JointRevoluteGraph & joint) const
    {
      if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
      {
        return pinocchio::JointModelRX();
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
      {
        return pinocchio::JointModelRY();
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
      {
        return pinocchio::JointModelRZ();
      }
      else
      {
        return pinocchio::JointModelRevoluteUnaligned(joint.axis);
      }
    }
    ReturnType operator()(const JointRevoluteUnboundedGraph & joint) const
    {
      if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
      {
        return pinocchio::JointModelRUBX();
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
      {
        return pinocchio::JointModelRUBY();
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
      {
        return pinocchio::JointModelRUBZ();
      }
      else
      {
        return pinocchio::JointModelRevoluteUnboundedUnaligned(joint.axis);
      }
    }
    ReturnType operator()(const JointPrismaticGraph & joint) const
    {
      if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
      {
        return pinocchio::JointModelPX();
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
      {
        return pinocchio::JointModelPY();
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
      {
        return pinocchio::JointModelPZ();
      }
      else
      {
        return pinocchio::JointModelPrismaticUnaligned(joint.axis);
      }
    }
    ReturnType operator()(const JointHelicalGraph & joint) const
    {
      if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
      {
        return pinocchio::JointModelHX(joint.pitch);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
      {
        return pinocchio::JointModelHY(joint.pitch);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
      {
        return pinocchio::JointModelHZ(joint.pitch);
      }
      else
      {
        return pinocchio::JointModelHelicalUnaligned(joint.axis, joint.pitch);
      }
    }
    ReturnType operator()(const JointFreeFlyerGraph & joint) const
    {
      return JointModelFreeFlyer();
    }
    ReturnType operator()(const JointTranslationGraph & joint) const
    {
      return JointModelTranslation();
    }
    ReturnType operator()(const JointPlanarGraph & joint) const
    {
      return JointModelPlanar();
    }
    ReturnType operator()(const JointSphericalGraph & joint) const
    {
      return JointModelSpherical();
    }
    ReturnType operator()(const JointSphericalZYXGraph & joint) const
    {
      return JointModelSphericalZYX();
    }
    ReturnType operator()(const JointUniversalGraph & joint) const
    {
      return JointModelUniversal(joint.axis1, joint.axis2);
    }

    ReturnType operator()(const JointMimicGraph & joint) const
    {
      return boost::apply_visitor(*this, joint.secondary_joint);
    }
    ReturnType operator()(const JointCompositeGraph & joint) const
    {
      JointModelComposite jmodel;
      for (size_t i = 0; i < joint.joints.size(); i++)
        jmodel.addJoint(boost::apply_visitor(*this, joint.joints[i]), joint.jointsPlacements[i]);

      return jmodel;
    }
  };

  struct AddJointModel : public boost::static_visitor<>
  {
    const ModelGraphVertex & source_vertex;
    const ModelGraphVertex & target_vertex;
    const ModelGraphEdge & edge;
    Model & model;
    CreateJointModel cjm;

    AddJointModel(
      const ModelGraphVertex & source,
      const ModelGraphVertex & target,
      const ModelGraphEdge & edge_,
      Model & model_)
    : source_vertex(source)
    , target_vertex(target)
    , edge(edge_)
    , model(model_)
    {
    }

    void addJointToModel(const JointModel & joint_model)
    {
      const pinocchio::SE3 & joint_pose = edge.out_to_joint;
      const pinocchio::SE3 & body_pose = edge.joint_to_in;

      const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
      JointIndex j_id = model.addJoint(
        previous_body.parentJoint, joint_model, previous_body.placement * joint_pose, edge.name);

      model.addJointFrame(j_id);
      model.appendBodyToJoint(j_id, target_vertex.inertia); // check this
      model.addBodyFrame(target_vertex.name, j_id, body_pose);
    }

    void operator()(const JointRevoluteGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointRevoluteUnboundedGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointPrismaticGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointHelicalGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointFreeFlyerGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointSphericalGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointSphericalZYXGraph & joint)
    {
      if (edge.reverse)
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "Graph - JointSphericalZYX cannot be reversed yet.");

      addJointToModel(cjm(joint));
    }

    void operator()(const JointPlanarGraph & joint)
    {
      if (edge.reverse)
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - JointPlanar cannot be reversed.");

      addJointToModel(cjm(joint));
    }

    void operator()(const JointTranslationGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointUniversalGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointCompositeGraph & joint)
    {
      addJointToModel(cjm(joint));
    }

    void operator()(const JointMimicGraph & joint)
    {
      if (edge.reverse)
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - JointMimic cannot be reversed.");

      if (!model.existJointName(joint.primary_name))
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error,
          "Graph - The parent joint of the mimic node is not in the kinematic tree");

      auto primary_joint = model.joints[model.getJointId(joint.primary_name)];
      addJointToModel(JointModelMimic(cjm(joint), primary_joint, joint.scaling, joint.offset));
    }

    void operator()(const JointFixedGraph & joint)
    {
      const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
      // Don't add a new joint in the model — create the fixed_joint frame
      FrameIndex f_id = model.addFrame(Frame(
        edge.name, previous_body.parentJoint, previous_body.placement * edge.out_to_joint,
        FIXED_JOINT, target_vertex.inertia));
      model.addBodyFrame(
        target_vertex.name, previous_body.parentJoint, edge.joint_to_in, (int)f_id);
    }
  };

  /// @brief Represents multibody model as a bidirectional graph.
  ///
  /// This is an intermediate step before creating a model, that
  /// allows more flexibility as to which body will be the root...
  struct ModelGraph
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
    void addBody(const std::string & vertex_name, const Inertia & inertia)
    {
      if (name_to_vertex.find(vertex_name) != name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - vertex already in graph");

      auto vertex_desc = boost::add_vertex(g);
      ModelGraphVertex & vertex = g[vertex_desc];
      vertex.name = vertex_name;
      vertex.inertia = inertia;

      name_to_vertex.insert({vertex_name, vertex_desc});
    }

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
      const SE3 & joint_to_in)
    {
      auto out_vertex = name_to_vertex.find(out_body);
      if (out_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - out_vertex does not exist");
      }
      auto in_vertex = name_to_vertex.find(in_body);
      if (in_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - in_vertex does not exist");
      }
      auto edge_desc = boost::add_edge(out_vertex->second, in_vertex->second, g);

      if (!edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "Graph - Edge cannot be added between these two vertexes");
      }
      ModelGraphEdge & edge = g[edge_desc.first];
      edge.name = joint_name;
      edge.joint = joint;
      edge.out_to_joint = out_to_joint;
      edge.joint_to_in = joint_to_in;

      auto reverse_edge_desc = boost::add_edge(in_vertex->second, out_vertex->second, g);
      if (!reverse_edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::runtime_error, "Graph - Reverse edge cannot be added between these two vertexes");
      }
      ModelGraphEdge & reverse_edge = g[reverse_edge_desc.first];
      reverse_edge.name = joint_name;
      reverse_edge.joint = boost::apply_visitor(ReverseJointVisitor(), joint);
      reverse_edge.out_to_joint = joint_to_in.inverse();
      reverse_edge.joint_to_in = out_to_joint.inverse();
      reverse_edge.reverse = true;
    }

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
      const std::string & root_joint_name = "root_joint") const
    {
      auto root_vertex = name_to_vertex.find(root_body);
      if (root_vertex == name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - root_body does not exist in the graph");

      std::vector<boost::default_color_type> colors(
        boost::num_vertices(g), boost::default_color_type::white_color);
      std::vector<EdgeDesc> edges;
      edges.reserve(boost::num_vertices(g));
      RecordTreeEdgeVisitor<Graph> tree_edge_visitor(&edges);
      boost::depth_first_search(g, tree_edge_visitor, colors.data(), root_vertex->second);

      Model model;
      const ModelGraphVertex & root_vertex_data = g[root_vertex->second];

      if (root_joint) // Root joint provided
      {
        JointIndex j_id = model.addJoint(
          0, boost::apply_visitor(CreateJointModel(), *root_joint), root_position, root_joint_name);
        model.addJointFrame(j_id);
        model.appendBodyToJoint(j_id, root_vertex_data.inertia);
        model.addBodyFrame(root_vertex_data.name, j_id);
      }
      else // Fixed to world
      {
        const Frame & parent_frame = model.frames[0];
        model.addFrame(Frame(
          root_vertex_data.name, parent_frame.parentJoint, 0,
          parent_frame.placement * root_position, BODY, root_vertex_data.inertia));
      }

      // Go through rest of the graph
      for (const EdgeDesc & edge_desc : edges)
      {
        const VertexDesc & source_vertex_desc = boost::source(edge_desc, g);
        const VertexDesc & target_vertex_desc = boost::target(edge_desc, g);
        const ModelGraphEdge & edge = g[edge_desc];
        const ModelGraphVertex & source_vertex = g[source_vertex_desc];
        const ModelGraphVertex & target_vertex = g[target_vertex_desc];

        AddJointModel visitor(source_vertex, target_vertex, edge, model);
        boost::apply_visitor(visitor, edge.joint);
      }
      return model;
    }

    void copyGraph(const ModelGraph & g, const std::string & prefix = "")
    {
      // Copy all vertices from g1
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.g[old_v];

        this->addBody(prefix + name, vertex_data.inertia);
      }

      // Copy all edges from g1
      for (auto e_it = boost::edges(g.g); e_it.first != e_it.second; ++e_it.first)
      {
        const auto & edge = *e_it.first;
        auto src = boost::source(edge, g.g);
        auto tgt = boost::target(edge, g.g);
        const auto & edge_data = g.g[edge];

        const auto & src_name = g.g[src].name;
        const auto & tgt_name = g.g[tgt].name;

        this->addJoint(
          prefix + edge_data.name, edge_data.joint, prefix + src_name, edge_data.out_to_joint,
          prefix + tgt_name, edge_data.joint_to_in);
      }
    }

    /// @brief Boost graph structure that holds the graph structure
    Graph g;
    /// @brief Name of the vertexes in the graph. Useful for graph parcours.
    std::unordered_map<std::string, VertexDesc> name_to_vertex;
  };

  ModelGraph mergeGraphs(
    const ModelGraph & g1,
    const ModelGraph & g2,
    const std::string & g1_body,
    const std::string & g2_body,
    const SE3 & pose_g2_body_in_g1,
    const boost::optional<JointGraphVariant> & merging_joint = boost::none,
    const std::string & merging_joint_name = "merging_joint",
    const std::string & g2_prefix = "g2/")
  {
    // Check bodies exists in graphs
    if (g1.name_to_vertex.find(g1_body) == g1.name_to_vertex.end())
      PINOCCHIO_THROW_PRETTY(std::runtime_error, "mergeGraph - g1_body not found");
    if (g2.name_to_vertex.find(g2_body) == g2.name_to_vertex.end())
      PINOCCHIO_THROW_PRETTY(std::runtime_error, "mergeGraph - g2_body not found");

    ModelGraph g_merged;

    g_merged.copyGraph(g1);
    g_merged.copyGraph(g2, g2_prefix);

    const std::string g2_body_merged = g2_prefix + g2_body;

    if (merging_joint)
      g_merged.addJoint(
        merging_joint_name, *merging_joint, g1_body, SE3::Identity(), g2_body_merged,
        pose_g2_body_in_g1);
    else
      g_merged.addJoint(
        merging_joint_name, JointFixedGraph(), g1_body, SE3::Identity(), g2_body_merged,
        pose_g2_body_in_g1);

    return g_merged;
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_graph_hpp__
