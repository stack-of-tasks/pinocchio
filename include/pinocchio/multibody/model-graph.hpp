//
// Copyright (c) 2024 INRIA
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
  namespace internal
  {
    template<typename Derived>
    struct JointBaseGraph
    {
      std::string name;

      JointBaseGraph(const std::string& n) : name(n) {};
    };

    struct JointFixedGraph : public JointBaseGraph<JointFixedGraph>
    {
      JointFixedGraph() : JointBaseGraph("") {};
      JointFixedGraph(const std::string& n) : JointBaseGraph(n){};
    };

    struct JointRevoluteGraph : public JointBaseGraph<JointRevoluteGraph>
    {
      Eigen::Vector3d axis;
      JointRevoluteGraph(const std::string& n, const Eigen::Vector3d& ax): JointBaseGraph(n), axis(ax) {};
    };

    struct JointPrismaticGraph : public JointBaseGraph<JointPrismaticGraph>
    {
      Eigen::Vector3d axis;
      JointPrismaticGraph(const std::string& n, const Eigen::Vector3d& ax): JointBaseGraph(n), axis(ax) {};
    };

    struct JointFreeFlyerGraph : public JointBaseGraph<JointFreeFlyerGraph>
    {
      JointFreeFlyerGraph() : JointBaseGraph("") {};
      JointFreeFlyerGraph(const std::string& n) : JointBaseGraph(n){};
    };

    using JointGraphVariant = boost::variant<
    JointFixedGraph,
    JointRevoluteGraph,
    JointPrismaticGraph,
    JointFreeFlyerGraph>;
    
    // TODO move in another header
    // TODO support all joints
    // TODO tests
    // TODO Redefine Rev{X,Y,Z} to use our options
    struct ReverseJointVisitor
    : public boost::static_visitor<JointGraphVariant>
    {
      typedef JointGraphVariant ReturnType;

      template<typename JointModelDerived>
      ReturnType operator()(const JointBaseGraph<JointModelDerived> &) const
      {
        throw std::runtime_error("TODO");
      }

      ReturnType
      operator()(const JointRevoluteGraph & joint) const
      {
        return JointRevoluteGraph(joint.name, -joint.axis);
      }

      ReturnType
      operator()(const JointPrismaticGraph & joint) const
      {
        return JointPrismaticGraph(joint.name, -joint.axis);
      }
      // ReturnType
      // operator()(const JointFixedGraph & joint) const
      // {
      //   return JointFixedGraph(joint.name);
      // }
      // ReturnType
      // operator()(const JointFreeFlyerGraph & joint) const
      // {
      //   return JointFreeFlyerGraph(joint.name);
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

      std::vector<EdgeDesc> *edges;
    };

  } // namespace internal

  struct ModelGraphVertex
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Body unique name
    std::string name;
    // Inertia at the CoM
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
    SE3 out_to_joint;
    // Transformation from joint to in vertex
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
      edge.joint = boost::apply_visitor(pinocchio::internal::ReverseJointVisitor(), joint);
      reverse_edge.out_to_joint = joint_to_in.inverse();
      reverse_edge.joint_to_in = out_to_joint.inverse();
    }

    Model buildModel(const std::string & root_body, const JointModel & root_joint) const
    {
      auto root_vertex = name_to_vertex.find(root_body);
      if (root_vertex == name_to_vertex.end())
      {
        throw std::runtime_error("TODO");
      }
      std::vector<boost::default_color_type> colors(
        boost::num_vertices(g), boost::default_color_type::white_color);
      std::vector<EdgeDesc> edges;
      edges.reserve(boost::num_vertices(g));
      internal::RecordTreeEdgeVisitor<Graph> tree_edge_visitor(&edges);
      boost::depth_first_search(g, tree_edge_visitor, colors.data(), root_vertex->second);

      Model model;
      model.addJoint(0, root_joint, SE3::Identity(), "root_joint");
      for (const EdgeDesc & edge_desc : edges)
      {
        VertexDesc source_vertex_desc = boost::source(edge_desc, g);
        VertexDesc target_vertex_desc = boost::target(edge_desc, g);
        const ModelGraphEdge & edge = g[edge_desc];
        const ModelGraphVertex & source_vertex = g[source_vertex_desc];
        const ModelGraphVertex & target_vertex = g[target_vertex_desc];
        std::cout << source_vertex.name << " -> " << edge.name << " -> " << target_vertex.name
                  << std::endl;
      }
      return model;
    }

    Graph g;
    std::unordered_map<std::string, VertexDesc> name_to_vertex;
    // TODO useless ?
    std::unordered_map<std::string, EdgeDesc> name_to_edge;
  };

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_graph_hpp__
