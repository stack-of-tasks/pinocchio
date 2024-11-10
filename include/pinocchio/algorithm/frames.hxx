//
// Copyright (c) 2015-2024 CNRS INRIA
//

#ifndef __pinocchio_algorithm_frames_hxx__
#define __pinocchio_algorithm_frames_hxx__

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline void updateFramePlacements(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::Frame Frame;
    typedef typename Model::FrameIndex FrameIndex;
    typedef typename Model::JointIndex JointIndex;

    // The following for loop starts by index 1 because the first frame is fixed
    // and corresponds to the universe.
    for (FrameIndex i = 1; i < (FrameIndex)model.nframes; ++i)
    {
      const Frame & frame = model.frames[i];
      const JointIndex & parent = frame.parentJoint;
      data.oMf[i] = data.oMi[parent] * frame.placement;
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline const typename DataTpl<Scalar, Options, JointCollectionTpl>::SE3 & updateFramePlacement(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const FrameIndex frame_id)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    const typename Model::Frame & frame = model.frames[frame_id];
    const typename Model::JointIndex & parent = frame.parentJoint;

    data.oMf[frame_id] = data.oMi[parent] * frame.placement;

    return data.oMf[frame_id];
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  inline void framesForwardKinematics(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q)
  {
    assert(model.check(data) && "data is not consistent with model.");

    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options> getFrameVelocity(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::Motion Motion;

    const typename Model::SE3 & oMi = data.oMi[joint_id];
    const typename Model::Motion & v = data.v[joint_id];
    switch (rf)
    {
    case LOCAL:
      return placement.actInv(v);
    case WORLD:
      return oMi.act(v);
    case LOCAL_WORLD_ALIGNED:
      return Motion(
        oMi.rotation() * (v.linear() + v.angular().cross(placement.translation())),
        oMi.rotation() * v.angular());
    default:
      throw std::invalid_argument("Bad reference frame.");
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options> getFrameAcceleration(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf)
  {
    PINOCCHIO_UNUSED_VARIABLE(model);
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::Motion Motion;

    const typename Model::SE3 & oMi = data.oMi[joint_id];
    const typename Model::Motion & a = data.a[joint_id];
    switch (rf)
    {
    case LOCAL:
      return placement.actInv(a);
    case WORLD:
      return oMi.act(a);
    case LOCAL_WORLD_ALIGNED:
      return Motion(
        oMi.rotation() * (a.linear() + a.angular().cross(placement.translation())),
        oMi.rotation() * a.angular());
    default:
      throw std::invalid_argument("Bad reference frame.");
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline MotionTpl<Scalar, Options> getFrameClassicalAcceleration(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame rf)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef MotionTpl<Scalar, Options> Motion;
    const Motion vel = getFrameVelocity(model, data, joint_id, placement, rf);
    Motion acc = getFrameAcceleration(model, data, joint_id, placement, rf);

    acc.linear() += vel.angular().cross(vel.linear());

    return acc;
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xLike>
  inline void getFrameJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const JointIndex joint_id,
    const SE3Tpl<Scalar, Options> & placement,
    const ReferenceFrame reference_frame,
    const Eigen::MatrixBase<Matrix6xLike> & J)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J.rows(), 6);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J.cols(), model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      joint_id < (JointIndex)model.njoints, "The index of the Joint is outside the bounds.");
    assert(model.check(data) && "data is not consistent with model.");

    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    const typename Data::SE3 oMframe = data.oMi[joint_id] * placement;
    details::translateJointJacobian(
      model, data, joint_id, reference_frame, oMframe, data.J, J.const_cast_derived());
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename Matrix6xLike>
  inline void computeFrameJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const FrameIndex frameId,
    const ReferenceFrame reference_frame,
    const Eigen::MatrixBase<Matrix6xLike> & J)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      J.cols(), model.nv,
      "The numbers of columns in the Jacobian matrix does not math the "
      "number of Dofs in the model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Model::Frame Frame;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::IndexVector IndexVector;

    const Frame & frame = model.frames[frameId];
    const JointIndex & joint_id = frame.parentJoint;

    const IndexVector & joint_support = model.supports[joint_id];

    switch (reference_frame)
    {
    case WORLD:
    case LOCAL_WORLD_ALIGNED: {
      typedef impl::JointJacobiansForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, Matrix6xLike>
        Pass;
      for (size_t k = 1; k < joint_support.size(); k++)
      {
        JointIndex parent = joint_support[k];
        Pass::run(
          model.joints[parent], data.joints[parent],
          typename Pass::ArgsType(model, data, q.derived(), J.const_cast_derived()));
      }

      if (reference_frame == LOCAL_WORLD_ALIGNED)
      {
        typename Data::SE3 & oMframe = data.oMf[frameId];
        oMframe = data.oMi[joint_id] * frame.placement;

        Matrix6xLike & J_ = J.const_cast_derived();

        const int colRef = nv(model.joints[joint_id]) + idx_v(model.joints[joint_id]) - 1;
        for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
        {
          typedef typename Matrix6xLike::ColXpr ColXprOut;
          MotionRef<ColXprOut> J_col(J_.col(j));

          J_col.linear() -= oMframe.translation().cross(J_col.angular());
        }
      }
      break;
    }
    case LOCAL: {
      data.iMf[joint_id] = frame.placement;

      typedef impl::JointJacobianForwardStep<
        Scalar, Options, JointCollectionTpl, ConfigVectorType, Matrix6xLike>
        Pass;
      for (JointIndex i = joint_id; i > 0; i = model.parents[i])
      {
        Pass::run(
          model.joints[i], data.joints[i],
          typename Pass::ArgsType(model, data, q.derived(), J.const_cast_derived()));
      }
      break;
    }
    default: {
      assert(false && "must never happened");
    }
    }
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xLike>
  void getFrameJacobianTimeVariation(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const FrameIndex frame_id,
    const ReferenceFrame rf,
    const Eigen::MatrixBase<Matrix6xLike> & dJ_)
  {
    assert(model.check(data) && "data is not consistent with model.");

    Matrix6xLike & dJ = dJ_.const_cast_derived();
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      dJ.cols(), model.nv,
      "The numbers of columns in the Jacobian matrix does not math the "
      "number of Dofs in the model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Model::Frame Frame;
    typedef typename Data::SE3 SE3;
    typedef typename SE3::Vector3 Vector3;
    typedef typename Data::Motion Motion;

    const Frame & frame = model.frames[frame_id];
    const JointIndex & joint_id = frame.parentJoint;

    typename Data::SE3 & oMframe = data.oMf[frame_id];
    oMframe = data.oMi[joint_id] * frame.placement;

    details::translateJointJacobian(
      model, data, joint_id, rf, oMframe, data.dJ, dJ.const_cast_derived());

    // Add contribution for LOCAL and LOCAL_WORLD_ALIGNED
    switch (rf)
    {
    case LOCAL: {
      const Motion & v_joint = data.v[joint_id];
      const Motion v_frame = frame.placement.actInv(v_joint);

      const int colRef = nv(model.joints[joint_id]) + idx_v(model.joints[joint_id]) - 1;
      for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
      {
        typedef typename Data::Matrix6x::ColXpr ColXprIn;
        typedef const MotionRef<ColXprIn> MotionIn;

        typedef typename Matrix6xLike::ColXpr ColXprOut;
        typedef MotionRef<ColXprOut> MotionOut;
        MotionIn v_in(data.J.col(j));
        MotionOut v_out(dJ.col(j));

        v_out -= v_frame.cross(oMframe.actInv(v_in));
      }
      break;
    }
    case LOCAL_WORLD_ALIGNED: {
      const Motion & ov_joint = data.ov[joint_id];
      const int colRef = nv(model.joints[joint_id]) + idx_v(model.joints[joint_id]) - 1;
      for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
      {
        typedef typename Data::Matrix6x::ColXpr ColXprIn;
        typedef const MotionRef<ColXprIn> MotionIn;

        typedef typename Matrix6xLike::ColXpr ColXprOut;
        typedef MotionRef<ColXprOut> MotionOut;
        MotionIn v_in(data.J.col(j));
        MotionOut v_out(dJ.col(j));

        v_out.linear() -=
          Vector3(ov_joint.linear() + ov_joint.angular().cross(oMframe.translation()))
            .cross(v_in.angular());
      }
      break;
    }

    case WORLD:
    default:
      break;
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  InertiaTpl<Scalar, Options> computeSupportedInertiaByFrame(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const FrameIndex frame_id,
    bool with_subtree)
  {
    assert(model.check(data) && "data is not consistent with model.");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef InertiaTpl<Scalar, Options> Inertia;

    const Frame & frame = model.frames[frame_id];
    const JointIndex & joint_id = frame.parentJoint;

    // Add all the inertia of child frames (i.e that are part of the same joint but comes after the
    // given frame)
    std::vector<typename Model::JointIndex> child_frames = {frame_id};
    Inertia I = frame.placement.act(frame.inertia); // Express the inertia in the parent joint frame
    for (FrameIndex i = frame_id + 1; i < (FrameIndex)model.nframes; ++i)
    {
      if (model.frames[i].parentJoint != joint_id)
        continue;
      if (
        std::find(child_frames.begin(), child_frames.end(), model.frames[i].parentFrame)
        == child_frames.end())
        continue;
      child_frames.push_back(i);
      I += model.frames[i].placement.act(model.frames[i].inertia);
    }

    if (!with_subtree)
    {
      return frame.placement.actInv(I);
    }

    // Express the inertia in the origin frame for simplicity.
    I = data.oMi[joint_id].act(I);

    // Add inertia of child joints
    const std::vector<typename Model::JointIndex> & subtree = model.subtrees[joint_id];
    for (size_t k = 1; k < subtree.size();
         ++k) // Skip the first joint as it is the one before the frame
    {
      const typename Model::JointIndex j_id = subtree[k];
      I += data.oMi[j_id].act(model.inertias[j_id]);
    }

    const pinocchio::SE3 oMf = data.oMi[joint_id] * frame.placement;
    return oMf.actInv(I);
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  ForceTpl<Scalar, Options> computeSupportedForceByFrame(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const FrameIndex frame_id)
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef InertiaTpl<Scalar, Options> Inertia;
    typedef MotionTpl<Scalar, Options> Motion;
    typedef ForceTpl<Scalar, Options> Force;

    const Frame & frame = model.frames[frame_id];
    const JointIndex & joint_id = frame.parentJoint;

    // Compute 'in body' forces
    const Inertia fI = computeSupportedInertiaByFrame(model, data, frame_id, false);
    const pinocchio::SE3 oMf = data.oMi[joint_id] * frame.placement;
    const Motion v = getFrameVelocity(model, data, frame_id, LOCAL);
    const Motion a = getFrameAcceleration(model, data, frame_id, LOCAL);
    Force f = fI.vxiv(v) + fI * (a - oMf.actInv(model.gravity));

    // Add child joints forces
    f = frame.placement.act(f); // Express force in parent joint frame
    const std::vector<typename Model::JointIndex> & subtree = model.subtrees[joint_id];
    for (size_t k = 1; k < subtree.size();
         ++k) // Skip the first joint as it is the one before the frame
    {
      const typename Model::JointIndex j_id = subtree[k];
      if (model.parents[j_id] != joint_id) // Joint is not a direct child
      {
        continue;
      }
      f += data.liMi[j_id].act(data.f[j_id]);
    }

    // Transform back to local frame
    return frame.placement.actInv(f);
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_frames_hxx__
