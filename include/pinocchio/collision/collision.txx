//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_collision_collision_txx__
#define __pinocchio_collision_collision_txx__

namespace pinocchio
{

  extern template PINOCCHIO_COLLISION_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI bool
  computeCollisions<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &,
    context::Data &,
    const GeometryModel &,
    GeometryData &,
    const Eigen::MatrixBase<context::VectorXs> &,
    const bool stopAtFirstCollision);

  extern template PINOCCHIO_COLLISION_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  computeBodyRadius<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, const GeometryModel &, GeometryData &);

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_collision_txx__
