//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/frames-derivatives.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameVelocityDerivatives<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameVelocityDerivatives<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameAccelerationDerivatives<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameAccelerationDerivatives<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameAccelerationDerivatives<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getFrameAccelerationDerivatives<
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
