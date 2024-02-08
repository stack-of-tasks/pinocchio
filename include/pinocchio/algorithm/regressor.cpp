//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/regressor.hpp"

namespace pinocchio {

  template void computeJointKinematicRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame, const SE3Tpl<context::Scalar,context::Options> &, const Eigen::MatrixBase<context::Matrix6xs> &);

  template context::Matrix6xs computeJointKinematicRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame, const SE3Tpl<context::Scalar,context::Options> &);

  template void computeJointKinematicRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);

  template context::Matrix6xs computeJointKinematicRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  template void computeFrameKinematicRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::Matrix6xs>
  (const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame, const Eigen::MatrixBase<context::Matrix6xs> &);

  template context::Matrix6xs computeFrameKinematicRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, const FrameIndex, const ReferenceFrame);

  template context::Matrix3x & computeStaticRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &);

  template void bodyRegressor
    <context::Motion, context::Motion, context::BodyRegressorType>
  (const MotionDense<context::Motion> &, const MotionDense<context::Motion> &, const Eigen::MatrixBase<context::BodyRegressorType> &);

  template context::BodyRegressorType bodyRegressor
    <context::Motion, context::Motion>
  (const MotionDense<context::Motion> &, const MotionDense<context::Motion> &);

  template context::BodyRegressorType & jointBodyRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, JointIndex);

  template context::BodyRegressorType & frameBodyRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, FrameIndex);

  template context::MatrixXs & computeJointTorqueRegressor
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs, context::VectorXs, context::VectorXs>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &, const Eigen::MatrixBase<context::VectorXs> &);
} // namespace pinocchio
