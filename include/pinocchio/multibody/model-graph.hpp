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
#include <unordered_map>

namespace pinocchio
{
  namespace internal
  {
    // TODO move in another header
    // TODO support all joints
    // TODO tests
    // TODO Redefine Rev{X,Y,Z} to use our options
    template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
    struct ReverseJointVisitorTpl
    : public boost::static_visitor<JointModelTpl<_Scalar, _Options, JointCollectionTpl>>
    {
      typedef _Scalar Scalar;
      enum
      {
        Options = _Options
      };

      typedef JointCollectionTpl<Scalar, Options> JointCollection;
      typedef typename JointCollection::JointModelVariant JointModelVariant;
      typedef JointModelVariant ReturnType;

      template<typename JointModelDerived>
      ReturnType operator()(const JointModelBase<JointModelDerived> &) const
      {
        throw std::runtime_error("TODO");
      }

      ReturnType
      operator()(const pinocchio::JointModelRevoluteUnalignedTpl<Scalar, Options> & joint) const
      {
        return JointModelRevoluteUnalignedTpl<Scalar, Options>(-joint.axis);
      }

      ReturnType operator()(const pinocchio::JointModelRX &) const
      {
        return JointModelRevoluteUnalignedTpl<Scalar, Options>(-1., 0., 0.);
      }

      ReturnType operator()(const pinocchio::JointModelRY &) const
      {
        return JointModelRevoluteUnalignedTpl<Scalar, Options>(0., -1., 0.);
      }

      ReturnType operator()(const pinocchio::JointModelRZ &) const
      {
        return JointModelRevoluteUnalignedTpl<Scalar, Options>(0., 0., -1.);
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

  } // namespace internal

  template<typename _Scalar, int _Options>
  struct ModelGraphVertexTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef InertiaTpl<Scalar, Options> Inertia;

    // Body unique name
    std::string name;
    // Inertia at the CoM
    Inertia inertia;
  };

  template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
  struct ModelGraphEdgeTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef JointCollectionTpl<Scalar, Options> JointCollection;
    typedef SE3Tpl<Scalar, Options> SE3;
    typedef JointModelTpl<Scalar, Options, JointCollectionTpl> JointModel;

    // Joint unique name
    std::string name;
    // Joint type
    JointModel joint;
    // Transformation from out vertex to joint
    SE3 out_to_joint;
    // Transformation from joint to in vertex
    SE3 joint_to_in;
  };

  template<typename _Scalar, int _Options, template<typename, int> class JointCollectionTpl>
  struct ModelGraphTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef _Scalar Scalar;
    enum
    {
      Options = _Options
    };

    typedef ModelGraphVertexTpl<Scalar, Options> ModelGraphVertex;
    typedef ModelGraphEdgeTpl<Scalar, Options, JointCollectionTpl> ModelGraphEdge;
    typedef InertiaTpl<Scalar, Options> Inertia;
    typedef SE3Tpl<Scalar, Options> SE3;
    typedef JointModelTpl<Scalar, Options, JointCollectionTpl> JointModel;
    typedef internal::ReverseJointVisitorTpl<Scalar, Options, JointCollectionTpl>
      ReverseJointVisitor;
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;

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
      const JointModel & joint,
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
      edge.joint = JointModel(boost::apply_visitor(ReverseJointVisitor(), joint));
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
