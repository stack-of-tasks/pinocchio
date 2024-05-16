//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_kinematics_txx__
#define __pinocchio_algorithm_kinematics_txx__

namespace pinocchio
{
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  updateGlobalPlacements<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void forwardKinematics<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void forwardKinematics<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void forwardKinematics<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
  } // namespace impl
  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getVelocity<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getClassicalAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_kinematics_txx__
