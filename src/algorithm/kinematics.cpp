//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/kinematics.hpp"

namespace pinocchio {
  
  template void updateGlobalPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
namespace impl {
  template void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs> > &);

  template void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

  template void forwardKinematics
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
} // namespace impl
  template MotionTpl<context::Scalar, context::Options> getVelocity
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  template MotionTpl<context::Scalar, context::Options> getClassicalAcceleration
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio 
