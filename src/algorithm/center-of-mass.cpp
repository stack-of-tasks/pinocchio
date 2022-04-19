//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/center-of-mass.hpp"

namespace pinocchio {

  template context::Scalar computeTotalMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &);

  template context::Scalar computeTotalMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);

  template void computeSubtreeMasses
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
namespace impl {
  template const context::Vector3 & centerOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const bool computeSubtreeComs);

  template const context::Vector3 & centerOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const bool computeSubtreeComs);

  template const context::Vector3 & centerOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const bool computeSubtreeComs);
} // namespace impl 
  template const context::Vector3 & centerOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, KinematicLevel, const bool computeSubtreeComs);

  template const context::Vector3 & centerOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &, const bool computeSubtreeComs);
namespace impl {
  template const context::Matrix3x & jacobianCenterOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const bool computeSubtreeComs);

  template void jacobianSubtreeCenterOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>, Eigen::Ref<context::Matrix3x>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &, const JointIndex &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

  template void jacobianSubtreeCenterOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<context::Matrix3x>>
  (const context::Model &, context::Data &, const JointIndex &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

  template void getJacobianSubtreeCenterOfMass
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<context::Matrix3x>>
  (const context::Model &, const context::Data &, const JointIndex &, const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);
} // namespace impl 
  template const context::Vector3 & getComFromCrba
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);

  template const context::Matrix3x & getJacobianComFromCrba
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, context::Data &);
} // namespace pinocchio
