//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_collision_distance_txx__
#define __pinocchio_collision_distance_txx__

namespace pinocchio
{

  extern template PINOCCHIO_COLLISION_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI std::size_t
  computeDistances<context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>(
    const context::Model &,
    context::Data &,
    const GeometryModel &,
    GeometryData &,
    const Eigen::MatrixBase<context::VectorXs> &);

} // namespace pinocchio

#endif // ifndef __pinocchio_collision_distance_txx__
