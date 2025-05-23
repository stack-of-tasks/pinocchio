//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/graph-visitor.hpp"

namespace pinocchio
{
  void ModelGraph::addBody(const std::string & vertex_name, const Inertia & inertia)
  {
    if (name_to_vertex.find(vertex_name) != name_to_vertex.end())
      PINOCCHIO_THROW_PRETTY(std::runtime_error, "Graph - vertex already in graph");

    auto vertex_desc = boost::add_vertex(g);
    ModelGraphVertex & vertex = g[vertex_desc];
    vertex.name = vertex_name;
    vertex.inertia = inertia;

    name_to_vertex.insert({vertex_name, vertex_desc});
  }

  void ModelGraph::addJoint(
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
    auto reversed_joint = boost::apply_visitor(ReverseJointVisitor(), joint);
    reverse_edge.joint = reversed_joint.first;
    reverse_edge.out_to_joint = joint_to_in.inverse();
    reverse_edge.joint_to_in = reversed_joint.second * out_to_joint.inverse();
    reverse_edge.reverse = true;
  }

  Model ModelGraph::buildModel(
    const std::string & root_body,
    const pinocchio::SE3 & root_position,
    const boost::optional<JointGraphVariant> & root_joint,
    const std::string & root_joint_name) const
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
        root_vertex_data.name, parent_frame.parentJoint, 0, parent_frame.placement * root_position,
        BODY, root_vertex_data.inertia));
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

  void ModelGraph::copyGraph(const ModelGraph & g, const std::string & prefix)
  {
    // Copy all vertices from g
    for (const auto & pair : g.name_to_vertex)
    {
      const auto & name = pair.first;
      const auto & old_v = pair.second;
      const auto & vertex_data = g.g[old_v];

      this->addBody(prefix + name, vertex_data.inertia);
    }

    // Copy all edges from g
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

  ModelGraph mergeGraphs(
    const ModelGraph & g1,
    const ModelGraph & g2,
    const std::string & g1_body,
    const std::string & g2_body,
    const SE3 & pose_g2_body_in_g1,
    const boost::optional<JointGraphVariant> & merging_joint,
    const std::string & merging_joint_name,
    const std::string & g2_prefix)
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
