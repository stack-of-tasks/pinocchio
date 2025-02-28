//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/kinematics.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  updateGlobalPlacements<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, context::Data &);
  namespace impl
  {
    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void forwardKinematics<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void forwardKinematics<
      context::Scalar,
      context::Options,
      JointCollectionDefaultTpl,
      Eigen::Ref<const context::VectorXs>,
      Eigen::Ref<const context::VectorXs>>(
      const context::Model &,
      context::Data &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &,
      const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);

    template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void forwardKinematics<
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

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    SE3Tpl<context::Scalar, context::Options>
    getRelativePlacement<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &,
      const context::Data &,
      const JointIndex,
      const JointIndex,
      const Convention);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getVelocity<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
    MotionTpl<context::Scalar, context::Options>
    getClassicalAcceleration<context::Scalar, context::Options, JointCollectionDefaultTpl>(
      const context::Model &, const context::Data &, const JointIndex, const ReferenceFrame);
} // namespace pinocchio
