//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/center-of-mass.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Scalar
  computeTotalMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Scalar
  computeTotalMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  computeSubtreeMasses<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Vector3 &
    centerOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const bool computeSubtreeComs);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Vector3 &
    centerOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const bool computeSubtreeComs);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Vector3 &
    centerOfMass<
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
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const bool computeSubtreeComs);
  } // namespace impl
  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Vector3 &
  centerOfMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, KinematicLevel, const bool computeSubtreeComs);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Vector3 &
  centerOfMass<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &, const bool computeSubtreeComs);
  namespace impl
  {
    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Matrix3x &
    jacobianCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const bool computeSubtreeComs);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void jacobianSubtreeCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const JointIndex &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void jacobianSubtreeCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      context::Data &,
      const JointIndex &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void getJacobianSubtreeCenterOfMass<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<context::Matrix3x>>(
      const context::Model &,
      const context::Data &,
      const JointIndex &,
      const Eigen::MatrixBase<Eigen::Ref<context::Matrix3x>> &);
  } // namespace impl
  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Vector3 &
  getComFromCrba<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const context::Matrix3x &
  getJacobianComFromCrba<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
} // namespace pinocchio
