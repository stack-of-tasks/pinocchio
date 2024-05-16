//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/frames.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  updateFramePlacements<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const
    SE3Tpl<context::Scalar, context::Options> &
    updateFramePlacement<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, context::Data &, const FrameIndex);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void framesForwardKinematics<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameVelocity<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameVelocity<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameClassicalAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameClassicalAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameJacobian<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const JointIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Matrix6xs
  getFrameJacobian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    context::Data &,
    const JointIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameJacobian<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Matrix6xs
  getFrameJacobian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeFrameJacobian<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeFrameJacobian<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const FrameIndex,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameJacobianTimeVariation<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &);
} // namespace pinocchio
