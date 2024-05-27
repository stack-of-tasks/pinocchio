//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_frames_derivatives_txx__
#define __pinocchio_algorithm_frames_derivatives_txx__

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getFrameVelocityDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs,
    context::Matrix6xs>(
    const context::Model &,
    const context::Data &,
    const JointIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getFrameVelocityDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getFrameAccelerationDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const JointIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getFrameAccelerationDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getFrameAccelerationDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const JointIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getFrameAccelerationDerivatives<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &,
    const Eigen::MatrixBase<context::Matrix6xs> &);

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_frames_derivatives_txx__
