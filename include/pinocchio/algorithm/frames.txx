//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_frames_txx__
#define __pinocchio_algorithm_frames_txx__

namespace pinocchio
{
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  updateFramePlacements<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI const
    SE3Tpl<context::Scalar, context::Options> &
    updateFramePlacement<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, context::Data &, const FrameIndex);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void framesForwardKinematics<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameVelocity<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameVelocity<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameClassicalAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getFrameClassicalAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const FrameIndex, const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void getFrameJacobian<
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

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Matrix6xs
  getFrameJacobian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    context::Data &,
    const JointIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void getFrameJacobian<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs>(
    const context::Model &,
    context::Data &,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI context::Matrix6xs
  getFrameJacobian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeFrameJacobian<
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

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void computeFrameJacobian<
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

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getFrameJacobianTimeVariation<
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

#endif // ifndef __pinocchio_algorithm_frames_txx__
