//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_parsers_graph_graph_visitor_hpp__
#define __pinocchio_parsers_graph_graph_visitor_hpp__

#include "pinocchio/macros.hpp"

#include "pinocchio/parsers/graph/fwd.hpp"

#include "pinocchio/multibody/joint/joint-collection.hpp"

#include "pinocchio/parsers/graph/model-graph.hpp"
#include "pinocchio/parsers/graph/joints.hpp"

#include <Eigen/Core>

#include <boost/graph/depth_first_search.hpp>

#include <boost/variant/static_visitor.hpp>
#include <boost/variant/apply_visitor.hpp>

#include <utility>
#include <stdexcept>

namespace pinocchio
{
  namespace graph
  {
    namespace internal
    {
      template<typename Derived>
      struct UpdateJointGraphPoseVisitorBase : public boost::static_visitor<SE3>
      {
        typedef Eigen::Ref<const Eigen::VectorXd> QRefType;

        QRefType q_ref;

        UpdateJointGraphPoseVisitorBase(const QRefType & q_ref)
        : q_ref(q_ref)
        {
        }

        template<typename JointModel>
        SE3 joint_calc(JointModel jmodel) const
        {
          return derived().joint_calc_impl(jmodel);
        }

        template<typename JointModel>
        SE3 joint_calc_default(JointModel jmodel) const
        {
          if (JointModel::NQ != q_ref.rows())
          {
            PINOCCHIO_THROW_PRETTY(
              std::invalid_argument, "Graph - q_ref mismatch joint configuration space");
          }
          jmodel.setIndexes(1, 0, 0);
          auto jdata = jmodel.createData();
          jmodel.calc(jdata, q_ref);

          return jdata.M;
        }

        const Derived & derived() const
        {
          return *static_cast<const Derived *>(this);
        }

        SE3 operator()(const JointFixed & joint) const
        {
          return joint.joint_offset;
        }

        SE3 operator()(const JointRevolute & joint) const
        {
          return joint_calc(JointModelRevoluteUnaligned(joint.axis));
        }

        SE3 operator()(const JointPrismatic & joint) const
        {
          return joint_calc(JointModelPrismaticUnaligned(joint.axis));
        }

        SE3 operator()(const JointRevoluteUnbounded & joint) const
        {
          return joint_calc(JointModelRevoluteUnboundedUnaligned(joint.axis));
        }

        SE3 operator()(const JointHelical & joint) const
        {
          return joint_calc(JointModelHelicalUnaligned(joint.axis, joint.pitch));
        }

        SE3 operator()(const JointUniversal & joint) const
        {
          return joint_calc(JointModelUniversal(joint.axis1, joint.axis2));
        }

        SE3 operator()(const JointFreeFlyer &) const
        {
          return joint_calc(JointModelFreeFlyer());
        }

        SE3 operator()(const JointSpherical &) const
        {
          return joint_calc(JointModelSpherical());
        }

        SE3 operator()(const JointSphericalZYX &) const
        {
          return joint_calc(JointModelSphericalZYX());
        }

        SE3 operator()(const JointPlanar &) const
        {
          return joint_calc(JointModelPlanar());
        }

        SE3 operator()(const JointTranslation &) const
        {
          return joint_calc(JointModelTranslation());
        }

        SE3 operator()(const JointMimic &) const
        {
          PINOCCHIO_THROW_PRETTY(
            std::invalid_argument,
            "Graph - Joint Mimic cannot have a q_ref. Please use the joint offset directly.");
        }
      };

      /// Return qref transformation.
      /// Also, modify composite joint internally to apply qref on it.
      struct UpdateJointGraphPoseVisitor
      : public UpdateJointGraphPoseVisitorBase<UpdateJointGraphPoseVisitor>
      {
        typedef UpdateJointGraphPoseVisitorBase<UpdateJointGraphPoseVisitor> Base;

        using Base::Base;

        template<typename JointModel>
        SE3 joint_calc_impl(JointModel jmodel) const
        {
          return Base::joint_calc_default(jmodel);
        }

        using Base::operator();
        SE3 operator()(JointComposite & joint) const
        {
          int index = 0;
          for (size_t i = 0; i < joint.joints.size(); i++)
          {
            int nq_curr =
              boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints[i]);
            auto u_temp = UpdateJointGraphPoseVisitor(q_ref.segment(index, nq_curr));
            SE3 pose_temp = boost::apply_visitor(u_temp, joint.joints[i]);
            joint.jointsPlacements[i] = joint.jointsPlacements[i] * pose_temp;
            index += nq_curr;
          }
          return SE3::Identity();
        }
      };

      /// Return reversed qref transformation.
      /// Also, modify composite joint internally to apply qref on it.
      struct UpdateJointGraphReversePoseVisitor
      : public UpdateJointGraphPoseVisitorBase<UpdateJointGraphReversePoseVisitor>
      {
        typedef UpdateJointGraphPoseVisitorBase<UpdateJointGraphReversePoseVisitor> Base;

        using Base::Base;

        template<typename JointModel>
        SE3 joint_calc_impl(JointModel jmodel) const
        {
          return Base::joint_calc_default(jmodel).inverse();
        }

        using Base::operator();
        SE3 operator()(const JointUniversal & joint) const
        {
          return Base::joint_calc_default(JointModelUniversal(-joint.axis2, -joint.axis1))
            .inverse();
        }

        SE3 operator()(JointComposite & joint) const
        {
          // Here, JointComposite is already reversed.
          // If we have the following forward composite joint:
          // [jointsPlacements[0]       jointPlacements[1]]
          //  jp0---qref0-----------j0---jp1---qref1----------j1
          //  With jpN is the original jointPlacements[N] and qref[N] the static transform computed
          //  with qref.
          //  We should have the following reversed one:
          // [jointsPlacements[0]          jointPlacements[1]]
          //            I-----------j1---qref^{-1}---xc1^{-1}------j0
          // And qref0^{-1} should be returned to forward this transformation in
          // joint_to_target.
          auto index = q_ref.rows();
          for (int i = 0; i < static_cast<int>(joint.joints.size()) - 1; i++)
          {
            int nq_curr = boost::apply_visitor(
              [](const auto & j_) { return j_.nq; }, joint.joints[static_cast<size_t>(i)]);
            index -= nq_curr;
            auto u_temp = UpdateJointGraphReversePoseVisitor(q_ref.segment(index, nq_curr));
            SE3 pose_temp = boost::apply_visitor(u_temp, joint.joints[static_cast<size_t>(i)]);
            joint.jointsPlacements[static_cast<size_t>(i + 1)] =
              pose_temp * joint.jointsPlacements[static_cast<size_t>(i + 1)];
          }
          int nq_curr =
            boost::apply_visitor([](const auto & j_) { return j_.nq; }, joint.joints.back());
          index -= nq_curr;
          auto u_temp = UpdateJointGraphReversePoseVisitor(q_ref.segment(index, nq_curr));
          SE3 pose_temp = boost::apply_visitor(u_temp, joint.joints.back());
          return pose_temp;
        }
      };
    } // namespace internal
  } // namespace graph
} // namespace pinocchio

#endif // ifndef __pinocchio_parsers_graph_graph_visitor_hpp__
