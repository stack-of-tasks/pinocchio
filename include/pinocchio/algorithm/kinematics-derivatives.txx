//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_kinematics_derivatives_txx__
#define __pinocchio_algorithm_kinematics_derivatives_txx__

namespace pinocchio
{
  namespace impl
  {

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    computeForwardKinematicsDerivatives<
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

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getJointVelocityDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getJointAccelerationDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getJointAccelerationDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>,
      Eigen::Ref<context::Matrix6xs>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix6xs>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getPointVelocityDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getPointClassicAccelerationDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

    extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
    getPointClassicAccelerationDerivatives<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const SE3Tpl<context::Scalar, context::Options> &,
      const ReferenceFrame,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);
  } // namespace impl

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  computeJointKinematicHessians<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  computeJointKinematicHessians<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  getJointKinematicHessian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const context::Data &,
    const JointIndex,
    const ReferenceFrame,
    Tensor<context::Scalar, 3, context::Options> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
    Tensor<context::Scalar, 3, context::Options>
    getJointKinematicHessian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_kinematics_derivatives_txx__
