//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_ALGORITHM_GEOMETRY

  #include "pinocchio/algorithm/geometry.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void updateGeometryPlacements<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &,
    context::Data &,
    const GeometryModel &,
    GeometryData &,
    const Eigen::MatrixBase<context::VectorXs> &);

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void
  updateGeometryPlacements<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, const context::Data &, const GeometryModel &, GeometryData &);

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_ALGORITHM_GEOMETRY
