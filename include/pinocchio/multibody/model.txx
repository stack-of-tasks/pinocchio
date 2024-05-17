//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_model_txx__
#define __pinocchio_multibody_model_txx__

#include "pinocchio/multibody/model.hpp"

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::ModelTpl();

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex, const JointModel &, const SE3 &, const std::string &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI JointIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJoint(
    const JointIndex,
    const JointModel &,
    const SE3 &,
    const std::string &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &,
    const context::VectorXs &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI JointIndex
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

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addJointFrame(
    const JointIndex &, int);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::appendBodyToJoint(
    const JointIndex, const Inertia &, const SE3 &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addBodyFrame(
    const std::string &, const JointIndex &, const SE3 &, int);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI FrameIndex
  ModelTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::addFrame(
    const Frame &, const bool);

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_model_txx__
