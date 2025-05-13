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

  using JointGraphVariant =
    boost::variant<JointFixedGraph, JointRevoluteGraph, JointPrismaticGraph, JointFreeFlyerGraph>;

  struct ReverseJointVisitor : public boost::static_visitor<JointGraphVariant>
  {
    typedef JointGraphVariant ReturnType;

    ReturnType operator()(const JointRevoluteGraph & joint) const
    {
      return JointRevoluteGraph(-joint.axis);
    }

    ReturnType operator()(const JointPrismaticGraph & joint) const
    {
      return JointPrismaticGraph(-joint.axis);
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

  struct ModelGraphVertex
  {
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
    // Joint unique name
    std::string name;
    // Joint type
    JointGraphVariant joint;
    // Transformation from out vertex to joint
    // transformation from body supporting to joint frame
    SE3 out_to_joint;
    // Transformation from joint to in vertex
    // transformation from joint frame to body supported
    SE3 joint_to_in;
  };

  struct AddJointModel : public boost::static_visitor<>
  {
    const ModelGraphVertex & source_vertex;
    const ModelGraphVertex & target_vertex;
    const ModelGraphEdge & edge;
    Model & model;

    AddJointModel(
      const ModelGraphVertex & source,
      const ModelGraphVertex & target,
      const ModelGraphEdge & edge_,
      Model & model_)
    : source_vertex(source)
    , target_vertex(target)
    , edge(edge_)
    , model(model_) {};

    void operator()(const JointRevoluteGraph & joint)
    {
      const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
      pinocchio::SE3 joint_pose = edge.out_to_joint;
      pinocchio::SE3 body_pose = edge.joint_to_in;
      pinocchio::JointIndex j_id;
      // Check joint axis
      if (joint.axis.isApprox(-Eigen::Vector3d::UnitX()))
      {
        pinocchio::SE3 transf = pinocchio::SE3(
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix(),
          Eigen::Vector3d::Zero());
        joint_pose.rotation() *= transf.rotation();
        body_pose = transf * body_pose;

        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelRX(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(-Eigen::Vector3d::UnitY()))
      {
        pinocchio::SE3 transf = pinocchio::SE3(
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(),
          Eigen::Vector3d::Zero());
        joint_pose.rotation() *= transf.rotation();
        body_pose = transf * body_pose;

        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelRY(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(-Eigen::Vector3d::UnitZ()))
      {
        pinocchio::SE3 transf = pinocchio::SE3(
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix(),
          Eigen::Vector3d::Zero());
        joint_pose.rotation() *= transf.rotation();
        body_pose = transf * body_pose;

        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelRZ(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelRX(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelRY(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelRZ(),
          previous_body.placement * joint_pose, edge.name);
      }
      else
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelRevoluteUnaligned(),
          previous_body.placement * joint_pose, edge.name);
      }
      model.addJointFrame(j_id);
      model.appendBodyToJoint(j_id, target_vertex.inertia); // Check this
      model.addBodyFrame(target_vertex.name, j_id, body_pose);
    }

    void operator()(const JointPrismaticGraph & joint)
    {
      const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
      pinocchio::SE3 joint_pose = edge.out_to_joint;
      pinocchio::SE3 body_pose = edge.joint_to_in;
      pinocchio::JointIndex j_id;
      // Check joint axis
      if (joint.axis.isApprox(-Eigen::Vector3d::UnitX()))
      {
        pinocchio::SE3 transf = pinocchio::SE3(
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix(),
          Eigen::Vector3d::Zero());
        joint_pose.rotation() *= transf.rotation();
        body_pose = transf * body_pose;

        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelPX(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(-Eigen::Vector3d::UnitY()))
      {
        pinocchio::SE3 transf = pinocchio::SE3(
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(),
          Eigen::Vector3d::Zero());
        joint_pose.rotation() *= transf.rotation();
        body_pose = transf * body_pose;

        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelPY(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(-Eigen::Vector3d::UnitZ()))
      {
        pinocchio::SE3 transf = pinocchio::SE3(
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix(),
          Eigen::Vector3d::Zero());
        joint_pose.rotation() *= transf.rotation();
        body_pose = transf * body_pose;

        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelPZ(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelPX(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelPY(),
          previous_body.placement * joint_pose, edge.name);
      }
      else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelPZ(),
          previous_body.placement * joint_pose, edge.name);
      }
      else
      {
        j_id = model.addJoint(
          previous_body.parentJoint, pinocchio::JointModelPrismaticUnaligned(),
          previous_body.placement * joint_pose, edge.name);
      }
      model.addJointFrame(j_id);
      model.appendBodyToJoint(j_id, target_vertex.inertia); // Check this
      model.addBodyFrame(target_vertex.name, j_id, body_pose);
    }

    void operator()(const JointFreeFlyerGraph & joint)
    {
      const Frame & previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
      pinocchio::SE3 joint_pose = edge.out_to_joint;
      pinocchio::SE3 body_pose = edge.joint_to_in;

      pinocchio::JointIndex j_id = model.addJoint(
        previous_body.parentJoint, pinocchio::JointModelFreeFlyer(),
        previous_body.placement * joint_pose, edge.name);

      model.addJointFrame(j_id);
      model.appendBodyToJoint(j_id, target_vertex.inertia); // Check this
      model.addBodyFrame(target_vertex.name, j_id, body_pose);
    }


      model.addJointFrame(j_id);
      model.appendBodyToJoint(j_id, target_vertex.inertia); // Check this
      model.addBodyFrame(target_vertex.name, j_id, body_pose);
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

  struct ModelGraph
  {
    typedef boost::
      adjacency_list<boost::vecS, boost::vecS, boost::directedS, ModelGraphVertex, ModelGraphEdge>
        Graph;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor VertexDesc;
    typedef typename boost::graph_traits<Graph>::edge_descriptor EdgeDesc;

    void addBody(const std::string & vertex_name, const Inertia & inertia)
    {
      if (name_to_vertex.find(vertex_name) != name_to_vertex.end())
        throw std::runtime_error("Graph - vertex already in graph");

      auto vertex_desc = boost::add_vertex(g);
      ModelGraphVertex & vertex = g[vertex_desc];
      vertex.name = vertex_name;
      vertex.inertia = inertia;

      name_to_vertex.insert({vertex_name, vertex_desc});
    }

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
        throw std::runtime_error("Graph - out vertex does not exist");
      }
      auto in_vertex = name_to_vertex.find(in_body);
      if (in_vertex == name_to_vertex.end())
      {
        throw std::runtime_error("Graph - in vertex does not exist");
      }
      auto edge_desc = boost::add_edge(out_vertex->second, in_vertex->second, g);

      //
      if (!edge_desc.second)
      {
        throw std::runtime_error("Graph - Edge cannot be added between these two vertexes");
      }
      ModelGraphEdge & edge = g[edge_desc.first];
      edge.name = joint_name;
      edge.joint = joint;
      edge.out_to_joint = out_to_joint;
      edge.joint_to_in = joint_to_in;

      auto reverse_edge_desc = boost::add_edge(in_vertex->second, out_vertex->second, g);
      if (!reverse_edge_desc.second)
      {
        throw std::runtime_error("Graph - reverse edge cannot be added");
      }
      ModelGraphEdge & reverse_edge = g[reverse_edge_desc.first];
      reverse_edge.name = joint_name;
      reverse_edge.joint = boost::apply_visitor(ReverseJointVisitor(), joint);
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

        AddJointModel visitor(source_vertex, target_vertex, edge, model);
        boost::apply_visitor(visitor, edge.joint);
      }
      return model;
    }

    // When root is a fixed joint
    Model buildModel(const std::string & root_body, const pinocchio::SE3 root_position) const
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
      const ModelGraphVertex & root_vertex_data = g[root_vertex->second];
      const Frame & parent_frame = model.frames[0];

      model.addFrame(Frame(
        root_vertex_data.name, parent_frame.parentJoint, 0, parent_frame.placement * root_position,
        BODY, root_vertex_data.inertia));

      // Go through rest of the graph
      for (const EdgeDesc & edge_desc : edges)
      {
        VertexDesc source_vertex_desc = boost::source(edge_desc, g);
        VertexDesc target_vertex_desc = boost::target(edge_desc, g);
        const ModelGraphEdge & edge = g[edge_desc];
        const ModelGraphVertex & source_vertex = g[source_vertex_desc];
        const ModelGraphVertex & target_vertex = g[target_vertex_desc];

        AddJointModel visitor(source_vertex, target_vertex, edge, model);
        boost::apply_visitor(visitor, edge.joint);
      }
      return model;
    }
    Graph g;
    std::unordered_map<std::string, VertexDesc> name_to_vertex;
  };
} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_graph_hpp__
