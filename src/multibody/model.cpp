//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/multibody/model.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::ModelTpl();

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex, const JointModel &, const SE3 &, const std::string &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex,
    const JointModel &,
    const SE3 &,
    const std::string &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex,
    const JointModel &,
    const SE3 &,
    const std::string &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJointFrame(
    const JointIndex &, int);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::appendBodyToJoint(
    const JointIndex, const Inertia &, const SE3 &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addBodyFrame(
    const std::string &, const JointIndex &, const SE3 &, int);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addFrame(
    const Frame &, const bool);

} // namespace pinocchio
