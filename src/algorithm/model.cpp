//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_MODEL

  #include "pinocchio/algorithm/model.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  appendModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const context::Model &,
    const FrameIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    context::Model &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Model
  appendModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const context::Model &,
    const FrameIndex,
    const SE3Tpl<context::Scalar, context::Options> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  appendModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const context::Model &,
    const GeometryModel &,
    const GeometryModel &,
    const FrameIndex,
    const SE3Tpl<context::Scalar, context::Options> &,
    context::Model &,
    GeometryModel &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &,
    const std::vector<JointIndex>,
    const Eigen::MatrixBase<context::VectorXs> &,
    context::Model &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI context::Model buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &,
    const std::vector<JointIndex> &,
    const Eigen::MatrixBase<context::VectorXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &,
    const GeometryModel &,
    const std::vector<JointIndex> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    context::Model &,
    GeometryModel &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void buildReducedModel<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    Eigen::aligned_allocator<GeometryModel>,
    context::VectorXs>(
    const context::Model &,
    const std::vector<GeometryModel, Eigen::aligned_allocator<GeometryModel>> &,
    const std::vector<JointIndex> &,
    const Eigen::MatrixBase<context::VectorXs> &,
    context::Model &,
    std::vector<GeometryModel, Eigen::aligned_allocator<GeometryModel>> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  transformJointIntoMimic<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const JointIndex &,
    const JointIndex &,
    const context::Scalar &,
    const context::Scalar &,
    context::Model &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  buildMimicModel<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &,
    const std::vector<JointIndex> &,
    const std::vector<JointIndex> &,
    const std::vector<context::Scalar> &,
    const std::vector<context::Scalar> &,
    context::Model &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI JointIndex
  findCommonAncestor<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, JointIndex, JointIndex, size_t &, size_t &);
} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_MODEL
