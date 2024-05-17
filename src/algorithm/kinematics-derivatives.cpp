//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/kinematics-derivatives.hpp"

namespace pinocchio
{
  namespace impl
  {

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getJointVelocityDerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getPointVelocityDerivatives<
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
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

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeJointKinematicHessians<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeJointKinematicHessians<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  getJointKinematicHessian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const context::Data &,
    const JointIndex,
    const ReferenceFrame,
    Tensor<context::Scalar, 3, context::Options> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    Tensor<context::Scalar, 3, context::Options>
    getJointKinematicHessian<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio
