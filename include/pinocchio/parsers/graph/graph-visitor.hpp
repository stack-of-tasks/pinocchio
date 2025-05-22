//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_graph_visitor_hpp__
#define __pinocchio_graph_visitor_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"

#include <boost/graph/visitors.hpp>

namespace pinocchio
{
  struct ReverseJointVisitor
  : public boost::static_visitor<std::pair<JointGraphVariant, pinocchio::SE3>>
  {
    using ReturnType = std::pair<JointGraphVariant, pinocchio::SE3>;

    ReturnType operator()(const JointRevoluteGraph & joint) const
    {
      return {JointRevoluteGraph(joint.axis, -joint.q_ref), pinocchio::SE3::Identity()};
    }

    ReturnType operator()(const JointRevoluteUnboundedGraph & joint) const
    {
      Eigen::Vector2d q_ref_rev;
      q_ref_rev << joint.q_ref[0], -joint.q_ref[1];

      return {JointRevoluteUnboundedGraph(joint.axis, q_ref_rev), pinocchio::SE3::Identity()};
    }

    ReturnType operator()(const JointPrismaticGraph & joint) const
    {
      return {JointPrismaticGraph(joint.axis, -joint.q_ref), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointFixedGraph & joint) const
    {
      return {JointFixedGraph(joint.joint_offset.inverse()), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointFreeFlyerGraph & joint) const
    {
      Eigen::VectorXd q_ref_rev = Eigen::VectorXd::Zero(7);
      Eigen::Quaterniond q_temp(joint.q_ref[6], joint.q_ref[3], joint.q_ref[4], joint.q_ref[5]);
      q_ref_rev << -joint.q_ref[0], -joint.q_ref[1], -joint.q_ref[2], q_temp.inverse().x(),
        q_temp.inverse().y(), q_temp.inverse().z(), q_temp.inverse().w();

      return {JointFreeFlyerGraph(q_ref_rev), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointSphericalGraph & joint) const
    {
      Eigen::VectorXd q_ref_rev = Eigen::VectorXd::Zero(4);
      Eigen::Quaterniond q_temp(joint.q_ref[3], joint.q_ref[0], joint.q_ref[1], joint.q_ref[2]);
      q_ref_rev << q_temp.inverse().x(), q_temp.inverse().y(), q_temp.inverse().z(),
        q_temp.inverse().w();

      return {JointSphericalGraph(q_ref_rev), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointSphericalZYXGraph & joint) const
    {
      // rotation matrix for spherique xyz for inverting spherical zyx
      Eigen::AngleAxisd Rx(-joint.q_ref[2], Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd Ry(-joint.q_ref[1], Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd Rz(-joint.q_ref[0], Eigen::Vector3d::UnitZ());
      // Eigen convention is right multiply
      Eigen::Matrix3d R = Rx.toRotationMatrix() * Ry.toRotationMatrix() * Rz.toRotationMatrix();
      // Convention it back into zyx
      Eigen::Vector3d q_reverse = R.eulerAngles(2, 1, 0);

      return {JointSphericalZYXGraph(q_reverse), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointTranslationGraph & joint) const
    {
      return {JointTranslationGraph(-joint.q_ref), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointPlanarGraph & joint) const
    {
      return {joint, pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointHelicalGraph & joint) const
    {
      return {JointHelicalGraph(joint.axis, joint.pitch, -joint.q_ref), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointUniversalGraph & joint) const
    {
      return {
        JointUniversalGraph(-joint.axis2, -joint.axis1, joint.q_ref), pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointMimicGraph & joint) const
    {
      return {joint, pinocchio::SE3::Identity()};
    }
    ReturnType operator()(const JointCompositeGraph & joint) const
    {
      JointCompositeGraph jReturn;
      auto temp = boost::apply_visitor(*this, joint.joints.back());
      jReturn.addJoint(temp.first, temp.second * pinocchio::SE3::Identity());
      // Reverse joints
      for (int i = static_cast<int>(joint.joints.size() - 2); i >= 0; i--)
      {
        temp = boost::apply_visitor(*this, joint.joints[i]);
        jReturn.addJoint(temp.first, temp.second * joint.jointsPlacements[i + 1].inverse());
      }
      return {jReturn, joint.jointsPlacements[0].inverse()};
    }
  };

  // Add const to every operator (otherwise call fails)
  struct CreateJointModel : public boost::static_visitor<JointModel>
  {
    typedef JointModel ReturnType;

    ReturnType operator()(const JointFixedGraph & /*joint*/) const
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
    ReturnType operator()(const JointFreeFlyerGraph & /*joint*/) const
    {
      return JointModelFreeFlyer();
    }
    ReturnType operator()(const JointTranslationGraph & /*joint*/) const
    {
      return JointModelTranslation();
    }
    ReturnType operator()(const JointPlanarGraph & /*joint*/) const
    {
      return JointModelPlanar();
    }
    ReturnType operator()(const JointSphericalGraph & /*joint*/) const
    {
      return JointModelSpherical();
    }
    ReturnType operator()(const JointSphericalZYXGraph & /*joint*/) const
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

    void operator()(const JointFixedGraph & /*joint*/)
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

} // namespace pinocchio

#endif // __pinocchio_graph_visitor_hpp__
