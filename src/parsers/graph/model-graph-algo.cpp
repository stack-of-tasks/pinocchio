//
// Copyright (c) 2025 INRIA
//
//
#include "pinocchio/parsers/graph/model-graph-algo.hpp"

#include "pinocchio/parsers/graph/fwd.hpp"
#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/frames.hpp"
#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/graph-visitor.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>

namespace pinocchio
{
  namespace graph
  {
    namespace
    {
      struct AddRootFrameVisitor : public boost::static_visitor<>
      {
        const ModelGraphVertex vertex;
        const SE3 position;
        const JointIndex j;
        Model & model;

        AddRootFrameVisitor(
          const ModelGraphVertex & v, const JointIndex & j_id, const SE3 & pose, Model & m)
        : vertex(v)
        , position(pose)
        , j(j_id)
        , model(m)
        {
        }

        void operator()(const BodyFrame & b_f) const
        {
          const FrameIndex f_id = model.getFrameId(model.names[j], JOINT);
          model.addFrame(Frame(vertex.name, j, f_id, position, BODY, b_f.inertia));
        }

        template<typename FrameGraph>
        void operator()(const FrameGraph & f_) const
        {
          const FrameIndex f_id = model.getFrameId(model.names[j], JOINT);
          model.addFrame(Frame(vertex.name, j, f_id, position, f_.f_type));
        }
      };

      struct CreateJointModelVisitor : public boost::static_visitor<JointModel>
      {
        typedef JointModelTpl<double> JointModel;
        typedef JointCollectionDefaultTpl<double> JointCollectionDefault;

        // Joint Revolute
        typedef typename JointCollectionDefault::JointModelRX JointModelRX;
        typedef typename JointCollectionDefault::JointModelRY JointModelRY;
        typedef typename JointCollectionDefault::JointModelRZ JointModelRZ;

        // Joint Revolute Unaligned
        typedef
          typename JointCollectionDefault::JointModelRevoluteUnaligned JointModelRevoluteUnaligned;

        // Joint Revolute UBounded
        typedef typename JointCollectionDefault::JointModelRUBX JointModelRUBX;
        typedef typename JointCollectionDefault::JointModelRUBY JointModelRUBY;
        typedef typename JointCollectionDefault::JointModelRUBZ JointModelRUBZ;

        // Joint Revolute Unbounded Unaligned
        typedef typename JointCollectionDefault::JointModelRevoluteUnboundedUnaligned
          JointModelRevoluteUnboundedUnaligned;

        // Joint Prismatic
        typedef typename JointCollectionDefault::JointModelPX JointModelPX;
        typedef typename JointCollectionDefault::JointModelPY JointModelPY;
        typedef typename JointCollectionDefault::JointModelPZ JointModelPZ;

        // Joint Prismatic Unaligned
        typedef typename JointCollectionDefault::JointModelPrismaticUnaligned
          JointModelPrismaticUnaligned;

        // Joint Spherical
        typedef typename JointCollectionDefault::JointModelSpherical JointModelSpherical;

        // Joint Spherical ZYX
        typedef typename JointCollectionDefault::JointModelSphericalZYX JointModelSphericalZYX;

        // Joint Translation
        typedef typename JointCollectionDefault::JointModelTranslation JointModelTranslation;

        // Joint FreeFlyer
        typedef typename JointCollectionDefault::JointModelFreeFlyer JointModelFreeFlyer;

        // Joint Planar
        typedef typename JointCollectionDefault::JointModelPlanar JointModelPlanar;

        // Joint Composite
        typedef typename JointCollectionDefault::JointModelComposite JointModelComposite;

        // Joint Mimic
        typedef typename JointCollectionDefault::JointModelMimic JointModelMimic;

        // Joint Helical
        typedef typename JointCollectionDefault::JointModelHx JointModelHx;
        typedef typename JointCollectionDefault::JointModelHy JointModelHy;
        typedef typename JointCollectionDefault::JointModelHz JointModelHz;

        // Joint Helical Unaligned
        typedef
          typename JointCollectionDefault::JointModelHelicalUnaligned JointModelHelicalUnaligned;

        // Joint Universal
        typedef typename JointCollectionDefault::JointModelUniversal JointModelUniversal;

        typedef JointModel ReturnType;

        ReturnType operator()(const JointFixed & /*joint*/) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - cannot create a fixed joint. In pinocchio, fixed joints are frame.");
        }
        ReturnType operator()(const JointRevolute & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelRX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelRY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelRZ();
          }
          else
          {
            return JointModelRevoluteUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointRevoluteUnbounded & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelRUBX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelRUBY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelRUBZ();
          }
          else
          {
            return JointModelRevoluteUnboundedUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointPrismatic & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelPX();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelPY();
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelPZ();
          }
          else
          {
            return JointModelPrismaticUnaligned(joint.axis);
          }
        }
        ReturnType operator()(const JointHelical & joint) const
        {
          if (joint.axis.isApprox(Eigen::Vector3d::UnitX()))
          {
            return JointModelHX(joint.pitch);
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitY()))
          {
            return JointModelHY(joint.pitch);
          }
          else if (joint.axis.isApprox(Eigen::Vector3d::UnitZ()))
          {
            return JointModelHZ(joint.pitch);
          }
          else
          {
            return JointModelHelicalUnaligned(joint.axis, joint.pitch);
          }
        }
        ReturnType operator()(const JointFreeFlyer & /*joint*/) const
        {
          return JointModelFreeFlyer();
        }
        ReturnType operator()(const JointTranslation & /*joint*/) const
        {
          return JointModelTranslation();
        }
        ReturnType operator()(const JointPlanar & /*joint*/) const
        {
          return JointModelPlanar();
        }
        ReturnType operator()(const JointSpherical & /*joint*/) const
        {
          return JointModelSpherical();
        }
        ReturnType operator()(const JointSphericalZYX & /*joint*/) const
        {
          return JointModelSphericalZYX();
        }
        ReturnType operator()(const JointUniversal & joint) const
        {
          return JointModelUniversal(joint.axis1, joint.axis2);
        }
        ReturnType operator()(const JointMimic & joint) const
        {
          return boost::apply_visitor(*this, joint.secondary_joint);
        }
        ReturnType operator()(const JointComposite & joint) const
        {
          JointModelComposite jmodel;
          for (size_t i = 0; i < joint.joints.size(); i++)
            jmodel.addJoint(
              boost::apply_visitor(*this, joint.joints[i]), joint.jointsPlacements[i]);

          return jmodel;
        }
      };

      struct AddJointModelVisitor : public boost::static_visitor<>
      {
        const ModelGraphVertex & source_vertex;
        const ModelGraphVertex & target_vertex;
        const ModelGraphEdge & edge;
        Model & model;
        CreateJointModelVisitor cjm;

        AddJointModelVisitor(
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

        template<typename JointGraph, typename FrameGraph>
        void operator()(const JointGraph & /*joint*/, const FrameGraph & /*f_*/)
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - Invalid joint between non body frames. Non body frames can "
            "only be added with Fixed joint");
        }

        template<typename JointGraph>
        void operator()(const JointGraph & joint, const BodyFrame & b_f)
        {
          if (boost::get<BodyFrame>(&source_vertex.frame) == nullptr)
          {
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Invalid joint between a body and a non body frame.");
          }

          const SE3 & joint_pose = edge.source_to_joint;
          const SE3 & body_pose = edge.joint_to_target;

          const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
          JointIndex j_id = model.addJoint(
            previous_body.parentJoint, cjm(joint), previous_body.placement * joint_pose, edge.name,
            edge.jlimit.maxEffort, edge.jlimit.maxVel, edge.jlimit.minConfig, edge.jlimit.maxConfig,
            edge.jlimit.friction, edge.jlimit.damping);

          model.addJointFrame(j_id);
          model.appendBodyToJoint(j_id, b_f.inertia); // check this
          model.addBodyFrame(target_vertex.name, j_id, body_pose);

          // armature
          model.armature.segment(model.joints[j_id].idx_v(), model.joints[j_id].nv()) =
            edge.jlimit.armature;
        }

        template<typename FrameGraph>
        void operator()(const JointFixed & joint, const FrameGraph & f_)
        {
          const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];

          model.addFrame(Frame(
            target_vertex.name, previous_body.parentJoint,
            previous_body.placement * edge.source_to_joint * joint.joint_offset
              * edge.joint_to_target,
            f_.f_type));
        }

        void operator()(const JointMimic & joint, const BodyFrame & b_f)
        {
          if (!edge.forward)
            PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - JointMimic cannot be reversed.");

          if (boost::get<BodyFrame>(&source_vertex.frame) == nullptr)
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - Invalid joint between a body and a non body frame.");

          if (!model.existJointName(joint.primary_name))
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument,
              "Graph - The parent joint of the mimic node is not in the kinematic tree");

          const auto primary_joint = model.joints[model.getJointId(joint.primary_name)];

          const SE3 & joint_pose = edge.source_to_joint;
          const SE3 & body_pose = edge.joint_to_target;

          const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
          JointIndex j_id = model.addJoint(
            previous_body.parentJoint,
            JointModelMimic(cjm(joint), primary_joint, joint.scaling, joint.offset),
            previous_body.placement * joint_pose, edge.name);

          model.addJointFrame(j_id);
          model.appendBodyToJoint(j_id, b_f.inertia); // check this
          model.addBodyFrame(target_vertex.name, j_id, body_pose);
        }

        void operator()(const JointFixed & joint, const BodyFrame & b_f)
        {
          // Need to check what's vertex the edge is coming from. If it's a body, then we add
          // both the fixed joint frame and a body frame. Otherwise, it's a "fake" fixed joint
          // That's only used for graph construction, so we just add the body frame.
          if (boost::get<BodyFrame>(&source_vertex.frame) == nullptr)
          {
            FrameIndex prev_f_id = model.getFrameId(source_vertex.name, OP_FRAME);
            if (prev_f_id == model.frames.size())
              prev_f_id = model.getFrameId(source_vertex.name, SENSOR);

            const Frame previous_frame = model.frames[prev_f_id];
            model.addFrame(Frame(
              target_vertex.name, previous_frame.parentJoint,
              previous_frame.placement * edge.source_to_joint * joint.joint_offset
                * edge.joint_to_target,
              BODY, b_f.inertia));
          }
          else
          {
            const Frame previous_body = model.frames[model.getFrameId(source_vertex.name, BODY)];
            // Don't add a new joint in the model — create the fixed_joint frame
            const FrameIndex f_id = model.addFrame(Frame(
              edge.name, previous_body.parentJoint,
              previous_body.placement * edge.source_to_joint * joint.joint_offset, FIXED_JOINT,
              b_f.inertia));
            SE3 body_placement = previous_body.placement * edge.source_to_joint * joint.joint_offset
                                 * edge.joint_to_target;
            model.addBodyFrame(
              target_vertex.name, previous_body.parentJoint, body_placement, (int)f_id);
          }
        }
      };

      struct RecordTreeEdgeVisitor : public boost::default_dfs_visitor
      {
        typedef ModelGraph::Graph Graph;
        typedef ModelGraph::EdgeDesc EdgeDesc;
        typedef std::unordered_map<std::string, bool> JointNameToDirection;

        RecordTreeEdgeVisitor(std::vector<EdgeDesc> * edges, JointNameToDirection * joint_forward)
        : edges(edges)
        , joint_forward(joint_forward)
        {
        }

        void tree_edge(EdgeDesc edge_desc, const Graph & g) const
        {
          const ModelGraphEdge & edge = g[edge_desc];
          (*joint_forward)[edge.name] = edge.forward;
          edges->push_back(edge_desc);
        }

        void forward_or_cross_edge(EdgeDesc, const Graph &) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument, "Graph - there is a cycle in the graph. It is not yet "
                                   "supported, please change graph construction.");
        }

        std::vector<EdgeDesc> * edges;
        /// Joint name to a bool that hold true if the joint is in forward direction
        JointNameToDirection * joint_forward;
      };
    } // namespace

    BuildModelWithBuildInfoReturn buildModelWithBuildInfo(
      const ModelGraph & g,
      const std::string & root_body,
      const SE3 & root_position,
      const JointVariant & root_joint,
      const std::string & root_joint_name)
    {
      BuildModelWithBuildInfoReturn ret;
      typedef ModelGraph::EdgeDesc EdgeDesc;

      auto root_vertex = g.name_to_vertex.find(root_body);
      if (root_vertex == g.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - root_body does not exist in the graph");

      std::vector<boost::default_color_type> colors(
        boost::num_vertices(g.graph), boost::default_color_type::white_color);
      std::vector<EdgeDesc> edges;
      edges.reserve(boost::num_vertices(g.graph));
      RecordTreeEdgeVisitor tree_edge_visitor(&edges, &ret.build_info._joint_forward);
      boost::depth_first_search(g.graph, tree_edge_visitor, colors.data(), root_vertex->second);

      Model & model = ret.model;
      const ModelGraphVertex & root_vertex_data = g.graph[root_vertex->second];

      if (!boost::get<JointFixed>(&root_joint)) // Root joint provided
      {
        ret.build_info._is_fixed = false;
        JointIndex j_id = model.addJoint(
          0, boost::apply_visitor(CreateJointModelVisitor(), root_joint), root_position,
          root_joint_name);
        model.addJointFrame(j_id);

        AddRootFrameVisitor afv(root_vertex_data, j_id, SE3::Identity(), model);
        boost::apply_visitor(afv, root_vertex_data.frame);
      }
      else // Fixed to world
      {
        ret.build_info._is_fixed = true;
        AddRootFrameVisitor afv(root_vertex_data, (JointIndex)0, root_position, model);
        boost::apply_visitor(afv, root_vertex_data.frame);
      }

      // Go through rest of the graph
      for (const auto & edge_desc : edges)
      {
        const auto & source_vertex_desc = boost::source(edge_desc, g.graph);
        const auto & target_vertex_desc = boost::target(edge_desc, g.graph);
        const ModelGraphEdge & edge = g.graph[edge_desc];
        const ModelGraphVertex & source_vertex = g.graph[source_vertex_desc];
        const ModelGraphVertex & target_vertex = g.graph[target_vertex_desc];

        AddJointModelVisitor visitor(source_vertex, target_vertex, edge, model);
        boost::apply_visitor(visitor, edge.joint, target_vertex.frame);
      }
      return ret;
    }

    Model buildModel(
      const ModelGraph & g,
      const std::string & root_body,
      const SE3 & root_position,
      const JointVariant & root_joint,
      const std::string & root_joint_name)
    {
      return buildModelWithBuildInfo(g, root_body, root_position, root_joint, root_joint_name)
        .model;
    }

    ModelGraph prefixNames(const ModelGraph & g, const std::string & prefix)
    {
      ModelGraph g_return;
      // Copy all vertices from g
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.graph[old_v];

        g_return.addFrame(prefix + name, vertex_data.frame);
      }

      // Copy all forward joints from g. Since addJoint will create the reverse edge, no need to
      // add both.
      for (auto e_it = boost::edges(g.graph); e_it.first != e_it.second; ++e_it.first)
      {
        const auto & edge = *e_it.first;
        const auto & edge_data = g.graph[edge];
        if (edge_data.forward)
        {
          auto src = boost::source(edge, g.graph);
          auto tgt = boost::target(edge, g.graph);

          const auto & src_name = g.graph[src].name;
          const auto & tgt_name = g.graph[tgt].name;

          g_return.edgeBuilder()
            .withName(prefix + edge_data.name)
            .withSourceVertex(prefix + src_name)
            .withSourcePose(edge_data.source_to_joint)
            .withTargetVertex(prefix + tgt_name)
            .withTargetPose(edge_data.joint_to_target)
            .withJointType(edge_data.joint)
            .build();
        }
      }
      return g_return;
    }

    ModelGraph merge(
      const ModelGraph & g1,
      const ModelGraph & g2,
      const std::string & g1_body,
      const std::string & g2_body,
      const SE3 & pose_g2_body_in_g1,
      const JointVariant & merging_joint,
      const std::string & merging_joint_name)
    {
      // Check bodies exists in graphs
      if (g1.name_to_vertex.find(g1_body) == g1.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "mergeGraph - g1_body not found");

      auto g1_vertex = g1.name_to_vertex.find(g1_body);
      if (boost::get<BodyFrame>(&g1.graph[g1_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument,
          "mergeGraph - Merging graphes needs to be done between two bodies. "
          "Vertex in g1 is not a body");

      if (g2.name_to_vertex.find(g2_body) == g2.name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "mergeGraph - g2_body not found");

      auto g2_vertex = g2.name_to_vertex.find(g2_body);
      if (boost::get<BodyFrame>(&g2.graph[g2_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument,
          "mergeGraph - Merging graphes needs to be done between two bodies. "
          "Vertex in g2 is not a body");

      ModelGraph g_merged;

      g_merged.appendGraph(g1);
      g_merged.appendGraph(g2);

      g_merged.edgeBuilder()
        .withName(merging_joint_name)
        .withSourceVertex(g1_body)
        .withTargetVertex(g2_body)
        .withTargetPose(pose_g2_body_in_g1)
        .withJointType(merging_joint)
        .build();

      return g_merged;
    }

    ModelGraph lockJoints(
      const ModelGraph & g,
      const std::vector<std::string> & joints_to_lock,
      const std::vector<Eigen::VectorXd> & reference_configurations)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        joints_to_lock.size() == reference_configurations.size(),
        "Graph - mismatch size between joints_to_lock list and reference configurations");

      ModelGraph g_locked;
      // Copy all vertices from g
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.graph[old_v];

        g_locked.addFrame(name, vertex_data.frame);
      }

      // Copy all forward joints from g. Since addJoint will create the reverse edge, no need to
      // add both.
      for (auto e_it = boost::edges(g.graph); e_it.first != e_it.second; ++e_it.first)
      {
        const auto & edge = *e_it.first;
        const auto & edge_data = g.graph[edge];
        if (edge_data.forward)
        {
          auto src = boost::source(edge, g.graph);
          auto tgt = boost::target(edge, g.graph);

          const auto & src_name = g.graph[src].name;
          const auto & tgt_name = g.graph[tgt].name;

          auto it = std::find(joints_to_lock.begin(), joints_to_lock.end(), edge_data.name);

          EdgeBuilder builder = g_locked.edgeBuilder()
                                  .withName(edge_data.name)
                                  .withSourceVertex(src_name)
                                  .withSourcePose(edge_data.source_to_joint)
                                  .withTargetVertex(tgt_name)
                                  .withTargetPose(edge_data.joint_to_target);

          // If joint should be locked we replaced it with a JointFixed with the right offset.
          if (it != joints_to_lock.end())
          {
            auto index = std::distance(joints_to_lock.begin(), it);
            const Eigen::VectorXd & q_ref =
              reference_configurations[static_cast<std::size_t>(index)];

            auto joint = edge_data.joint;
            SE3 pose_offset =
              boost::apply_visitor(internal::UpdateJointGraphPoseVisitor(q_ref), joint);

            builder.withJointType(JointFixed(pose_offset));
          }
          else
          {
            builder.withJointType(edge_data.joint);
          }

          builder.build();
        }
      }
      return g_locked;
    }
  } // namespace graph
} // namespace pinocchio
