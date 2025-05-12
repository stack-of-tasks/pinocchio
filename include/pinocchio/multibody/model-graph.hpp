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
    template<typename Derived>
    struct JointBaseGraph
    {
      std::string name;

      JointBaseGraph(const std::string & n)
      : name(n) {};
    };

    struct JointFixedGraph : public JointBaseGraph<JointFixedGraph>
    {
      JointFixedGraph()
      : JointBaseGraph("") {};
      JointFixedGraph(const std::string & n)
      : JointBaseGraph(n) {};
    };

    struct JointRevoluteGraph : public JointBaseGraph<JointRevoluteGraph>
    {
      // rotation axis
      Eigen::Vector3d axis;
      // axis to flip around if needed
      Eigen::Vector3d flip_axis;
      // if axis is unitary (X, Y, Z) and negative, then flipped is true, to remember to flip
      // position when adding joint to model
      bool flipped = false;
      JointRevoluteGraph(const std::string & n, const Eigen::Vector3d & ax, const bool flip = false)
      : JointBaseGraph(n)
      , axis(ax)
      , flipped(flip)
      {
        if (axis.isApprox(-Eigen::Vector3d::UnitX()))
        {
          flipped = true;
          axis = Eigen::Vector3d::UnitX();
          flip_axis = Eigen::Vector3d::UnitY();
        }
        else if (axis.isApprox(-Eigen::Vector3d::UnitY()))
        {
          flipped = true;
          axis = Eigen::Vector3d::UnitY();
          flip_axis = Eigen::Vector3d::UnitX();
        }
        else if (axis.isApprox(-Eigen::Vector3d::UnitZ()))
        {
          flipped = true;
          axis = Eigen::Vector3d::UnitZ();
          flip_axis = Eigen::Vector3d::UnitY();
        }
      };
    };

    struct JointPrismaticGraph : public JointBaseGraph<JointPrismaticGraph>
    {
      Eigen::Vector3d axis;
      JointPrismaticGraph(const std::string & n, const Eigen::Vector3d & ax)
      : JointBaseGraph(n)
      , axis(ax) {};
    };

    struct JointFreeFlyerGraph : public JointBaseGraph<JointFreeFlyerGraph>
    {
      JointFreeFlyerGraph()
      : JointBaseGraph("") {};
      JointFreeFlyerGraph(const std::string & n)
      : JointBaseGraph(n) {};
    };

    using JointGraphVariant =
      boost::variant<JointFixedGraph, JointRevoluteGraph, JointPrismaticGraph, JointFreeFlyerGraph>;

    // TODO move in another header
    // TODO support all joints
    // TODO tests
    // TODO Redefine Rev{X,Y,Z} to use our options
    struct ReverseJointVisitor : public boost::static_visitor<JointGraphVariant>
    {
      typedef JointGraphVariant ReturnType;

      template<typename JointModelDerived>
      ReturnType operator()(const JointBaseGraph<JointModelDerived> &) const
      {
        throw std::runtime_error("TODO");
      }

      ReturnType operator()(const JointRevoluteGraph & joint) const
      {
        return JointRevoluteGraph(joint.name, -joint.axis);
      }

      ReturnType operator()(const JointPrismaticGraph & joint) const
      {
        return JointPrismaticGraph(joint.name, -joint.axis);
      }
      ReturnType operator()(const JointFixedGraph & joint) const
      {
        return joint;
      }
      ReturnType operator()(const JointFreeFlyerGraph & joint) const
      {
        return joint;
      }
    };

    struct createJointModel : public boost::static_visitor<JointModel>
    {
      typedef JointModel ReturnType;

      template<typename JointModelDerived>
      ReturnType operator()(const JointBaseGraph<JointModelDerived> &) const
      {
        throw std::runtime_error("TODO");
      }

      ReturnType operator()(const JointRevoluteGraph & joint) const
      {
        const Eigen::Vector3d & axis = joint.axis;

        if (axis.isApprox(Eigen::Vector3d::UnitX()))
          return pinocchio::JointModelRX();
        else if (axis.isApprox(Eigen::Vector3d::UnitY()))
          return pinocchio::JointModelRY();
        else if (axis.isApprox(Eigen::Vector3d::UnitZ()))
          return pinocchio::JointModelRZ();
        else
          return pinocchio::JointModelRevoluteUnaligned(axis.normalized());
      }

      ReturnType operator()(const JointPrismaticGraph & joint) const
      {
        const Eigen::Vector3d & axis = joint.axis;

        if (axis.isApprox(Eigen::Vector3d::UnitX()))
          return pinocchio::JointModelPX();
        else if (axis.isApprox(Eigen::Vector3d::UnitY()))
          return pinocchio::JointModelPY();
        else if (axis.isApprox(Eigen::Vector3d::UnitZ()))
          return pinocchio::JointModelPZ();
        else
          return pinocchio::JointModelPrismaticUnaligned(joint.axis.normalized());
      }

      ReturnType operator()(const JointFreeFlyerGraph &) const
      {
        return pinocchio::JointModelFreeFlyer();
      }
      // ReturnType
      // operator()(const JointFreeFlyerGraph& joint) const
      // {
      //   return joint;
      // }
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

  } // namespace internal

  struct ModelGraphVertex
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::unordered_map<std::string, pinocchio::SE3> com_pos_wrt_edges;
    // Body unique name
    std::string name;
    // Inertia at the CoM
    // CoM position is defined wrt to a body frame
    // Since we reverse, how do we keep this ? Many joints
    // Inertia matrix also, no ?
    Inertia inertia;
  };

  struct ModelGraphEdge
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Joint unique name
    std::string name;
    // Joint type
    internal::JointGraphVariant joint;
    // Transformation from out vertex to joint
    // transformation from body supporting to joint frame
    SE3 out_to_joint;
    // Transformation from joint to in vertex
    // transformation from joint frame to body supported
    SE3 joint_to_in;
  };

  struct ModelGraph
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef boost::
      adjacency_list<boost::vecS, boost::vecS, boost::directedS, ModelGraphVertex, ModelGraphEdge>
        Graph;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexDesc;
    typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

    void addBody(const std::string & vertex_name, const Inertia & inertia)
    {
      auto vertex_desc = boost::add_vertex(g);
      ModelGraphVertex & vertex = g[vertex_desc];
      vertex.name = vertex_name;
      vertex.inertia = inertia;
      if (!name_to_vertex.insert({vertex_name, vertex_desc}).second)
      {
        throw std::runtime_error("TODO");
      }
    }

    void addJoint(
      const std::string & joint_name,
      const internal::JointGraphVariant & joint,
      const std::string & out_body,
      const SE3 & out_to_joint,
      const std::string & in_body,
      const SE3 & joint_to_in)
    {
      auto out_vertex = name_to_vertex.find(out_body);
      if (out_vertex == name_to_vertex.end())
      {
        throw std::runtime_error("TODO");
      }
      auto in_vertex = name_to_vertex.find(in_body);
      if (in_vertex == name_to_vertex.end())
      {
        throw std::runtime_error("TODO");
      }
      auto edge_desc = boost::add_edge(out_vertex->second, in_vertex->second, g);

      //
      if (!edge_desc.second)
      {
        throw std::runtime_error("TODO");
      }
      ModelGraphEdge & edge = g[edge_desc.first];
      edge.name = joint_name;
      edge.joint = joint;
      edge.out_to_joint = out_to_joint;
      edge.joint_to_in = joint_to_in;

      auto reverse_edge_desc = boost::add_edge(in_vertex->second, out_vertex->second, g);
      if (!reverse_edge_desc.second)
      {
        throw std::runtime_error("TODO");
      }
      ModelGraphEdge & reverse_edge = g[reverse_edge_desc.first];
      reverse_edge.name = joint_name + std::string("_reverse");
      reverse_edge.joint = boost::apply_visitor(pinocchio::internal::ReverseJointVisitor(), joint);
      reverse_edge.out_to_joint = joint_to_in.inverse();
      reverse_edge.joint_to_in = out_to_joint.inverse();
    }

    Model buildModel(
      const std::string & root_body,
      const pinocchio::SE3 root_position,
      const JointModel & root_joint) const
    {
      auto root_vertex = name_to_vertex.find(root_body);
      if (root_vertex == name_to_vertex.end())
      {
        throw std::runtime_error("Graph - root body does not exist in the graph");
      }
      std::vector<boost::default_color_type> colors(
        boost::num_vertices(g), boost::default_color_type::white_color);
      std::vector<EdgeDesc> edges;
      edges.reserve(boost::num_vertices(g));
      RecordTreeEdgeVisitor<Graph> tree_edge_visitor(&edges);
      boost::depth_first_search(g, tree_edge_visitor, colors.data(), root_vertex->second);

      Model model;
      JointIndex j_id = model.addJoint(0, root_joint, root_position, "root_joint");
      model.addJointFrame(j_id);

      // add root body and glue it on root_joint
      const ModelGraphVertex & root_vertex_data = g[root_vertex->second];
      model.appendBodyToJoint(j_id, root_vertex_data.inertia);
      model.addBodyFrame(root_vertex_data.name, j_id);
      // Go through rest of the graph
      for (const EdgeDesc & edge_desc : edges)
      {
        VertexDesc source_vertex_desc = boost::source(edge_desc, g);
        VertexDesc target_vertex_desc = boost::target(edge_desc, g);
        const ModelGraphEdge & edge = g[edge_desc];
        const ModelGraphVertex & source_vertex = g[source_vertex_desc];
        const ModelGraphVertex & target_vertex = g[target_vertex_desc];

        // Get previous body frame and get right joint placement, according to it
        const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];

        if (boost::get<pinocchio::internal::JointFixedGraph>(&edge.joint))
        {
          // Don't add a new joint in the model — create the fixed_joint frame
          FrameIndex f_id = model.addFrame(Frame(
            edge.name, j_id, previous_body.placement * edge.out_to_joint, FIXED_JOINT,
            target_vertex.inertia));
          model.addBodyFrame(target_vertex.name, j_id, edge.joint_to_in, (int)f_id);
          continue;
        }
        pinocchio::SE3 joint_pose = edge.out_to_joint;
        pinocchio::SE3 body_pose = edge.joint_to_in;
        if (
          const pinocchio::internal::JointRevoluteGraph * jrg =
            boost::get<pinocchio::internal::JointRevoluteGraph>(&edge.joint))
        {
          if (jrg->flipped)
          {
            pinocchio::SE3 transf = pinocchio::SE3(
              Eigen::AngleAxisd(M_PI, jrg->flip_axis).toRotationMatrix(), Eigen::Vector3d::Zero());
            joint_pose.rotation() *= transf.rotation();
            body_pose = transf * body_pose;
          }
        }
        j_id = model.addJoint(
          j_id, boost::apply_visitor(pinocchio::internal::createJointModel(), edge.joint),
          previous_body.placement * joint_pose, edge.name);
        model.addJointFrame(j_id);
        model.appendBodyToJoint(j_id, target_vertex.inertia);
        model.addBodyFrame(target_vertex.name, j_id, body_pose);
      }
      return model;
    }

    Graph g;
    std::unordered_map<std::string, VertexDesc> name_to_vertex;
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_graph_hpp__
