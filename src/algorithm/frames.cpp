//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/frames.hpp"

namespace pinocchio {

  template void updateFramePlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);

  template const SE3Tpl<context::Scalar, context::Options> & updateFramePlacement
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, const FrameIndex);

  template void framesForwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

  template MotionTpl<context::Scalar, context::Options> getFrameVelocity
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const JointIndex, const SE3Tpl<context::Scalar, context::Options> &, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getFrameVelocity
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getFrameAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const JointIndex, const SE3Tpl<context::Scalar, context::Options> &, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getFrameAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getFrameClassicalAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const JointIndex, const SE3Tpl<context::Scalar, context::Options> &, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getFrameClassicalAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  template void getFrameJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const context::Model &, context::Data &, const JointIndex, const SE3Tpl<context::Scalar, context::Options> &, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);

  template context::Matrix6xs getFrameJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, const JointIndex, const SE3Tpl<context::Scalar, context::Options> &, const ReferenceFrame);

  template void getFrameJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);

  template context::Matrix6xs getFrameJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame);

  template void computeFrameJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::Matrix6xs>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const FrameIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);

  template void computeFrameJacobian
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::Matrix6xs>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const FrameIndex, const Eigen::MatrixBase<context::Matrix6xs> &);

  template void getFrameJacobianTimeVariation
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);
} // namespace pinocchio
