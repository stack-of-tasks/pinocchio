//
// Copyright (c) 2022-2024 INRIA
//

#include "pinocchio/algorithm/regressor.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeJointKinematicRegressor<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs>(
    const context::Model &,
    const context::Data &,
    const JointIndex,
    const ReferenceFrame,
    const SE3Tpl<context::Scalar, context::Options> &,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Matrix6xs
  computeJointKinematicRegressor<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const context::Data &,
    const JointIndex,
    const ReferenceFrame,
    const SE3Tpl<context::Scalar, context::Options> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeJointKinematicRegressor<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::Matrix6xs>(
    const context::Model &,
    const context::Data &,
    const JointIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<context::Matrix6xs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Matrix6xs
  computeJointKinematicRegressor<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void computeFrameKinematicRegressor<
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
  computeFrameKinematicRegressor<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Matrix3x &
  computeStaticRegressor<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  bodyRegressor<context::Motion, context::Motion, context::BodyRegressorType>(
    const MotionDense<context::Motion> &,
    const MotionDense<context::Motion> &,
    const Eigen::MatrixBase<context::BodyRegressorType> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::BodyRegressorType
  bodyRegressor<context::Motion, context::Motion>(
    const MotionDense<context::Motion> &, const MotionDense<context::Motion> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::BodyRegressorType &
  jointBodyRegressor<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, JointIndex);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::BodyRegressorType &
  frameBodyRegressor<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, FrameIndex);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::MatrixXs &
  computeJointTorqueRegressor<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::VectorXs,
    context::VectorXs>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Data::RowVectorXs &
  computeKineticEnergyRegressor<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs,
    context::VectorXs>(
    const context::Model &,
    context::Data &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const Eigen::MatrixBase<context::VectorXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Data::RowVectorXs &
  computePotentialEnergyRegressor<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

} // namespace pinocchio
