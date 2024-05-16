//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_geometry_txx__
#define __pinocchio_algorithm_geometry_txx__

#ifndef PINOCCHIO_SKIP_CASADI_UNSUPPORTED

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void updateGeometryPlacements<
    context::Scalar,
    context::Options,
    JointCollectionDefaultTpl,
    context::VectorXs>(
    const context::Model &,
    context::Data &,
    const GeometryModel &,
    GeometryData &,
    const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI void
  updateGeometryPlacements<context::Scalar, context::Options, JointCollectionDefaultTpl>(
    const context::Model &, const context::Data &, const GeometryModel &, GeometryData &);

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_CASADI_UNSUPPORTED

#endif // ifndef __pinocchio_algorithm_geometry_txx__
