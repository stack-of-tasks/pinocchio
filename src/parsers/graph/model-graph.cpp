//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/parsers/graph/model-graph.hpp"

#include "pinocchio/parsers/graph/fwd.hpp"

#include "pinocchio/parsers/graph/joints.hpp"
#include "pinocchio/parsers/graph/frames.hpp"
#include "pinocchio/parsers/graph/graph-visitor.hpp"

#include <boost/graph/adjacency_list.hpp>

#include <boost/variant/apply_visitor.hpp>

namespace pinocchio
{
  namespace graph
  {
    namespace
    {
      /// \return true if \p joint_name is already used in \p graph
      bool isJointNameExists(const ModelGraph::Graph & graph, const std::string & joint_name)
      {
        for (auto e_it = boost::edges(graph); e_it.first != e_it.second; ++e_it.first)
        {
          if (graph[*e_it.first].name == joint_name)
          {
            return true;
          }
        }
        return false;
      }

      struct ReverseJointGraphVisitor : public boost::static_visitor<std::pair<JointVariant, SE3>>
      {
        using ReturnType = std::pair<JointVariant, SE3>;

        ReturnType operator()(const JointRevolute & joint) const
        {
          return {JointRevolute(joint.axis), SE3::Identity()};
        }
        ReturnType operator()(const JointRevoluteUnbounded & joint) const
        {
          return {JointRevoluteUnbounded(joint.axis), SE3::Identity()};
        }
        ReturnType operator()(const JointPrismatic & joint) const
        {
          return {JointPrismatic(joint.axis), SE3::Identity()};
        }
        ReturnType operator()(const JointFixed & joint) const
        {
          return {JointFixed(joint.joint_offset.inverse()), SE3::Identity()};
        }
        ReturnType operator()(const JointFreeFlyer &) const
        {
          return {JointFreeFlyer(), SE3::Identity()};
        }
        ReturnType operator()(const JointSpherical &) const
        {
          return {JointSpherical(), SE3::Identity()};
        }
        ReturnType operator()(const JointSphericalZYX &) const
        {
          return {JointSphericalZYX(), SE3::Identity()};
        }
        ReturnType operator()(const JointTranslation &) const
        {
          return {JointTranslation(), SE3::Identity()};
        }
        ReturnType operator()(const JointPlanar &) const
        {
          return {JointPlanar(), SE3::Identity()};
        }
        ReturnType operator()(const JointHelical & joint) const
        {
          return {JointHelical(joint.axis, joint.pitch), SE3::Identity()};
        }
        ReturnType operator()(const JointUniversal & joint) const
        {
          return {JointUniversal(-joint.axis2, -joint.axis1), SE3::Identity()};
        }
        ReturnType operator()(const JointMimic & joint) const
        {
          return {joint, SE3::Identity()};
        }
        ReturnType operator()(const JointComposite & joint) const
        {
          JointComposite jReturn;
          auto temp = boost::apply_visitor(*this, joint.joints.back());
          jReturn.addJoint(temp.first, temp.second * SE3::Identity());
          // Reverse joints
          for (int i = static_cast<int>(joint.joints.size()) - 2; i >= 0; i--)
          {
            temp = boost::apply_visitor(*this, joint.joints[static_cast<size_t>(i)]);
            jReturn.addJoint(
              temp.first,
              temp.second * joint.jointsPlacements[static_cast<size_t>(i + 1)].inverse());
          }
          return {jReturn, joint.jointsPlacements[0].inverse()};
        }
      };

      struct MakeJointLimitsDefaultVisitor : public boost::static_visitor<JointLimits>
      {
        template<typename Joint>
        JointLimits operator()(const Joint &) const
        {
          JointLimits jlimit;
          jlimit.setDimensions<Joint::nq, Joint::nv>();
          return jlimit;
        }

        JointLimits operator()(const JointComposite & j) const
        {
          JointLimits jlimit = boost::apply_visitor(*this, j.joints[0]);

          for (size_t i = 1; i < j.joints.size(); i++)
          {
            int nq = boost::apply_visitor([](const auto & j) { return j.nq; }, j.joints[i]);
            int nv = boost::apply_visitor([](const auto & j) { return j.nv; }, j.joints[i]);
            jlimit.append(boost::apply_visitor(*this, j.joints[i]), nq, nv);
          }
          return jlimit;
        }
      };

      struct ReverseJointLimitsVisitor : public boost::static_visitor<JointLimits>
      {
        const JointLimits jlimit;

        explicit ReverseJointLimitsVisitor(const JointLimits & jlimit)
        : jlimit(jlimit)
        {
        }

        // For revolute, revolute unbounded, prismatic, helical, translation, mimic, fixed
        template<typename Joint>
        JointLimits operator()(const Joint &) const
        {
          JointLimits jlimit_return = jlimit;
          jlimit_return.maxConfig = -jlimit.minConfig;
          jlimit_return.minConfig = -jlimit.maxConfig;

          return jlimit_return;
        }

        // For freeFlyer = no changes, except for translation limits. where min becomes -max and
        // inverse
        JointLimits operator()(const JointFreeFlyer &) const
        {
          JointLimits jlimit_return = jlimit;
          jlimit_return.maxConfig.segment(0, 3) = -jlimit.minConfig.segment(0, 3);
          jlimit_return.minConfig.segment(0, 3) = -jlimit.maxConfig.segment(0, 3);

          return jlimit;
        }

        // For spherical = no changes
        JointLimits operator()(const JointSpherical &) const
        {
          return jlimit;
        }

        // For planar = rotation is unbounded so we only do the translation
        JointLimits operator()(const JointPlanar &) const
        {
          JointLimits jlimit_return = jlimit;
          jlimit_return.maxConfig.segment(0, 2) = -jlimit.minConfig.segment(0, 2);
          jlimit_return.minConfig.segment(0, 2) = -jlimit.maxConfig.segment(0, 2);

          return jlimit_return;
        }

        // universal = inverse axis config, so inverse limits. Only 2 axis
        JointLimits operator()(const JointUniversal &) const
        {
          JointLimits jlimit_return = jlimit;
          std::swap(jlimit_return.maxConfig[0], jlimit_return.maxConfig[1]);
          std::swap(jlimit_return.minConfig[0], jlimit_return.minConfig[1]);

          std::swap(jlimit_return.maxEffort[0], jlimit_return.maxEffort[1]);
          std::swap(jlimit_return.maxVel[0], jlimit_return.maxVel[1]);
          std::swap(jlimit_return.friction[0], jlimit_return.friction[1]);
          std::swap(jlimit_return.damping[0], jlimit_return.damping[1]);
          std::swap(jlimit_return.armature[0], jlimit_return.armature[1]);

          return jlimit_return;
        }

        // ZYX = inverse order and max becomes min,
        JointLimits operator()(const JointSphericalZYX & j) const
        {
          JointLimits jlimit_return = jlimit;
          // inverse z limits with x limits
          std::swap(jlimit_return.maxEffort[0], jlimit_return.maxEffort[2]);
          std::swap(jlimit_return.maxVel[0], jlimit_return.maxVel[2]);
          std::swap(jlimit_return.friction[0], jlimit_return.friction[2]);
          std::swap(jlimit_return.damping[0], jlimit_return.damping[2]);
          std::swap(jlimit_return.armature[0], jlimit_return.armature[2]);

          for (int i = 0; i < j.nq; i++)
          {
            jlimit_return.maxConfig[i] = -jlimit.minConfig[j.nq - 1 - i];
            jlimit_return.minConfig[i] = -jlimit.maxConfig[j.nq - 1 - i];
          }

          return jlimit_return;
        }

        // Composite = inverse order inside and apply visitor on each joint
        JointLimits operator()(const JointComposite & j) const
        {
          int nq_curr =
            boost::apply_visitor([](const auto & j_) { return j_.nq; }, j.joints.back());
          int index_back_config = j.nq - nq_curr;
          int nv_curr =
            boost::apply_visitor([](const auto & j_) { return j_.nv; }, j.joints.back());
          int index_back_tangent = j.nv - nv_curr;

          int i = static_cast<int>(j.joints.size() - 1);

          auto createAndFillJointLimits = [&](
                                            int nq_curr, int nv_curr, int index_back_config,
                                            int index_back_tangent) -> JointLimits {
            // Step 1: Initialize jtemp using the visitor
            JointLimits jtemp = boost::apply_visitor(
              MakeJointLimitsDefaultVisitor(), j.joints[static_cast<size_t>(i)]);

            jtemp.minConfig.conservativeResize(nq_curr);
            jtemp.maxConfig.conservativeResize(nq_curr);
            jtemp.maxEffort.conservativeResize(nv_curr);
            jtemp.maxVel.conservativeResize(nv_curr);
            jtemp.friction.conservativeResize(nv_curr);
            jtemp.damping.conservativeResize(nv_curr);
            jtemp.armature.conservativeResize(nv_curr);

            // Step 2: Copy segments from jlimit into jtemp
            jtemp.minConfig.segment(0, nq_curr) =
              jlimit.minConfig.segment(index_back_config, nq_curr);
            jtemp.maxConfig.segment(0, nq_curr) =
              jlimit.maxConfig.segment(index_back_config, nq_curr);

            jtemp.maxEffort.segment(0, nv_curr) =
              jlimit.maxEffort.segment(index_back_tangent, nv_curr);
            jtemp.maxVel.segment(0, nv_curr) = jlimit.maxVel.segment(index_back_tangent, nv_curr);

            jtemp.friction.segment(0, nv_curr) =
              jlimit.friction.segment(index_back_tangent, nv_curr);
            jtemp.damping.segment(0, nv_curr) = jlimit.damping.segment(index_back_tangent, nv_curr);

            jtemp.armature.segment(0, nv_curr) =
              jlimit.armature.segment(index_back_tangent, nv_curr);

            return jtemp;
          };

          JointLimits jtemp =
            createAndFillJointLimits(nq_curr, nv_curr, index_back_config, index_back_tangent);

          JointLimits jlimit_return =
            boost::apply_visitor(ReverseJointLimitsVisitor(jtemp), j.joints.back());
          // Do the same for the rest
          for (i = static_cast<int>(j.joints.size() - 2); i >= 0; i--)
          {
            nq_curr = boost::apply_visitor(
              [](const auto & j_) { return j_.nq; }, j.joints[static_cast<size_t>(i)]);
            index_back_config -= nq_curr;
            nv_curr = boost::apply_visitor(
              [](const auto & j_) { return j_.nv; }, j.joints[static_cast<size_t>(i)]);
            index_back_tangent -= nv_curr;

            JointLimits jtemp_ =
              createAndFillJointLimits(nq_curr, nv_curr, index_back_config, index_back_tangent);
            jlimit_return.append(
              boost::apply_visitor(
                ReverseJointLimitsVisitor(jtemp_), j.joints[static_cast<size_t>(i)]),
              nq_curr, nv_curr);
          }
          return jlimit_return;
        }
      };
    } // namespace

    template<int Nq, int Nv>
    void JointLimits::setDimensions()
    {
      const double infty = std::numeric_limits<double>::infinity();

      maxEffort = Eigen::VectorXd::Constant(Nv, infty);
      maxVel = Eigen::VectorXd::Constant(Nv, infty);
      maxConfig = Eigen::VectorXd::Constant(Nq, infty);
      minConfig = Eigen::VectorXd::Constant(Nq, -infty);
      friction = Eigen::VectorXd::Constant(Nv, 0.);
      damping = Eigen::VectorXd::Constant(Nv, 0.);
      armature = Eigen::VectorXd::Constant(Nv, 0);
    }

    void JointLimits::append(const JointLimits & range, const int nq, const int nv)
    {
      assert(range.maxEffort.size() == nv);
      assert(range.minConfig.size() == nq);

      maxEffort.conservativeResize(maxEffort.size() + nv);
      maxEffort.tail(nv) = range.maxEffort;
      maxVel.conservativeResize(maxVel.size() + nv);
      maxVel.tail(nv) = range.maxVel;

      minConfig.conservativeResize(minConfig.size() + nq);
      minConfig.tail(nq) = range.minConfig;
      maxConfig.conservativeResize(maxConfig.size() + nq);
      maxConfig.tail(nq) = range.maxConfig;

      damping.conservativeResize(damping.size() + nv);
      damping.tail(nv) = range.damping;
      friction.conservativeResize(friction.size() + nv);
      friction.tail(nv) = range.friction;

      armature.conservativeResize(armature.size() + nv);
      armature.tail(nv) = range.armature;
    }

    EdgeParameters::EdgeParameters(
      const std::string & jname,
      const std::string & source_name,
      const SE3 & source_to_joint,
      const std::string & target_name,
      const SE3 & joint_to_target,
      const JointVariant & joint,
      const boost::optional<Eigen::VectorXd> q_ref)
    : name(jname)
    , source_vertex(source_name)
    , source_to_joint(source_to_joint)
    , target_vertex(target_name)
    , joint_to_target(joint_to_target)
    , joint(joint)
    , q_ref(q_ref)
    , jlimit(boost::apply_visitor(MakeJointLimitsDefaultVisitor(), joint))
    {
    }

    void EdgeBuilder::build()
    {
      // fill jointLimits
      param.jlimit = boost::apply_visitor(MakeJointLimitsDefaultVisitor(), param.joint);

      auto assignIfPresent = [&](auto & target, const auto & source) {
        if (source)
        {
          target = *source;
        }
      };

      assignIfPresent(param.jlimit.maxConfig, maxConfig);
      assignIfPresent(param.jlimit.minConfig, minConfig);
      assignIfPresent(param.jlimit.maxVel, maxVel);
      assignIfPresent(param.jlimit.maxEffort, maxEffort);
      assignIfPresent(param.jlimit.friction, friction);
      assignIfPresent(param.jlimit.damping, damping);
      assignIfPresent(param.jlimit.armature, armature);

      param.jlimit.frictionLoss = frictionLoss;

      g.addJoint(param);
    }

    void ModelGraph::addFrame(const std::string & vertex_name, const FrameVariant & frame)
    {
      if (name_to_vertex.find(vertex_name) != name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - vertex already in graph");

      auto vertex_desc = boost::add_vertex(graph);
      ModelGraphVertex & vertex = graph[vertex_desc];
      vertex.name = vertex_name;

      vertex.frame = frame;
      name_to_vertex.insert({vertex_name, vertex_desc});
    }

    void ModelGraph::addBody(const std::string & vertex_name, const Inertia & inert)
    {
      addFrame(vertex_name, BodyFrame(inert));
    }

    GeometryBuilder ModelGraph::geometryBuilder()
    {
      return GeometryBuilder(*this);
    }

    void ModelGraph::addGeometry(const std::string & vertex_name, const Geometry & geom)
    {
      auto vertex_n = name_to_vertex.find(vertex_name);
      if (vertex_n == name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Vertex does not exist. Cannot add geometry");

      ModelGraphVertex & vertex_desc = graph[vertex_n->second];

      if (boost::get<BodyFrame>(&vertex_desc.frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Geometries can only be added to bodies");

      vertex_desc.addGeometry(geom);
    }

    void
    ModelGraph::addGeometries(const std::string & vertex_name, const std::vector<Geometry> & geoms)
    {
      auto vertex_n = name_to_vertex.find(vertex_name);
      if (vertex_n == name_to_vertex.end())
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Vertex does not exist. Cannot add geometry");

      ModelGraphVertex & vertex_desc = graph[vertex_n->second];

      if (boost::get<BodyFrame>(&vertex_desc.frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Geometries can only be added to bodies");

      for (const auto & g : geoms)
        vertex_desc.addGeometry(g);
    }

    void ModelGraph::addJoint(const EdgeParameters & params)
    {
      auto source_vertex = name_to_vertex.find(params.source_vertex);
      if (source_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - source_vertex does not exists");
      }
      auto target_vertex = name_to_vertex.find(params.target_vertex);
      if (target_vertex == name_to_vertex.end())
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - target_vertex does not exists");
      }
      if (isJointNameExists(graph, params.name))
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Graph - joint_name already exists");
      }
      if (boost::edge(source_vertex->second, target_vertex->second, graph).second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Joint already connect source_body to target_body");
      }
      if (boost::get<BodyFrame>(&graph[source_vertex->second].frame) == nullptr)
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - sensor and op_frame can only be appended to bodies");

      auto edge_desc = boost::add_edge(source_vertex->second, target_vertex->second, graph);
      if (!edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Edge cannot be added between these two vertices");
      }

      ModelGraphEdge & edge = graph[edge_desc.first];
      edge.name = params.name;
      edge.joint = params.joint;
      /// When there is no qref we will have the following transformation:
      /// source_vertex --- source_to_joint --- joint --- joint_to_target --- target_vertex
      /// If qref is defined we should modify source_to_joint:
      /// source_vertex --- source_to_joint --- qref --- joint --- joint_to_target --- target_vertex
      if (params.q_ref)
      {
        edge.source_to_joint =
          params.source_to_joint
          * boost::apply_visitor(internal::UpdateJointGraphPoseVisitor(*params.q_ref), edge.joint);
      }
      else
      {
        edge.source_to_joint = params.source_to_joint;
      }

      edge.joint_to_target = params.joint_to_target;

      edge.jlimit = params.jlimit;

      auto reverse_edge_desc = boost::add_edge(target_vertex->second, source_vertex->second, graph);
      if (!reverse_edge_desc.second)
      {
        PINOCCHIO_THROW_PRETTY(
          std::invalid_argument, "Graph - Reverse edge cannot be added between these two vertices");
      }

      ModelGraphEdge & reverse_edge = graph[reverse_edge_desc.first];
      reverse_edge.name = params.name;
      auto reversed_joint = boost::apply_visitor(ReverseJointGraphVisitor(), params.joint);
      reverse_edge.joint = reversed_joint.first;
      /// When there is no qref we will have the following transformation:
      ///                (reverse_edge.source_to_joint)         (reverse_edge.joint_to_target)
      /// target_vertex --- joint_to_target^{-1} --- joint^{-1} --- source_to_joint^{-1} ---
      /// source_vertex
      ///
      /// Since some joint, once reversed can add a static transform joint_static (JointComposite,
      /// first jointsPlacements must connect the joint to his source)
      ///
      /// target_vertex --- joint_to_target^{-1} --- joint^{-1} --- joint_static ---
      /// source_to_joint^{-1} --- source_vertex
      ///
      /// When qref is defined will have the following transformation:
      /// target_vertex --- joint_to_target^{-1} --- joint^{-1} --- qref^{-1}--- joint_static ---
      /// source_to_joint^{-1} --- source_vertex
      if (params.q_ref)
      {
        reverse_edge.joint_to_target =
          boost::apply_visitor(
            internal::UpdateJointGraphReversePoseVisitor(*params.q_ref), reverse_edge.joint)
          * reversed_joint.second * params.source_to_joint.inverse();
      }
      else
      {
        reverse_edge.joint_to_target = reversed_joint.second * params.source_to_joint.inverse();
      }

      reverse_edge.source_to_joint = params.joint_to_target.inverse();
      reverse_edge.forward = false;

      reverse_edge.jlimit =
        boost::apply_visitor(ReverseJointLimitsVisitor(params.jlimit), params.joint);
    }

    void ModelGraph::addJoint(
      const std::string & joint_name,
      const JointVariant & joint,
      const std::string & source_body,
      const SE3 & source_to_joint,
      const std::string & target_body,
      const SE3 & joint_to_target)
    {
      return addJoint(EdgeParameters(
        joint_name, source_body, source_to_joint, target_body, joint_to_target, joint));
    }

    EdgeBuilder ModelGraph::edgeBuilder()
    {
      return EdgeBuilder(*this);
    }

    void ModelGraph::appendGraph(const ModelGraph & g)
    {
      // Copy all vertices from g
      for (const auto & pair : g.name_to_vertex)
      {
        const auto & name = pair.first;
        const auto & old_v = pair.second;
        const auto & vertex_data = g.graph[old_v];

        this->addFrame(name, vertex_data.frame);
      }

      // Copy all forward joints from g. Since addJoint will create the reverse edge, no need to add
      // both.
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

          this->edgeBuilder()
            .withName(edge_data.name)
            .withSourceVertex(src_name)
            .withSourcePose(edge_data.source_to_joint)
            .withTargetVertex(tgt_name)
            .withTargetPose(edge_data.joint_to_target)
            .withJointType(edge_data.joint)
            .build();
        }
      }
    }
  } // namespace graph
} // namespace pinocchio
