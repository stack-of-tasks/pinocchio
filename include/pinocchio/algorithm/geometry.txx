//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_geometry_txx__
#define __pinocchio_algorithm_geometry_txx__

#ifndef PINOCCHIO_SKIP_CASADI_UNSUPPORTED

namespace pinocchio {

  extern template PINOCCHIO_DLLAPI void updateGeometryPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
    (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_DLLAPI void updateGeometryPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, const context::Data &, const GeometryModel &, GeometryData &);

#ifdef PINOCCHIO_WITH_HPP_FCL
#ifndef PINOCCHIO_SKIP_ALGORITHM_GEOMETRY

  extern template PINOCCHIO_DLLAPI bool computeCollisions
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
    (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &, const bool stopAtFirstCollision);

  extern template PINOCCHIO_DLLAPI std::size_t computeDistances
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
    (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_DLLAPI void computeBodyRadius
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, const GeometryModel &, GeometryData &);

#endif // PINOCCHIO_SKIP_ALGORITHM_GEOMETRY
#endif // PINOCCHIO_WITH_HPP_FCL

} // namespace pinocchio

#endif // PINOCCHIO_SKIP_CASADI_UNSUPPORTED

#endif // ifndef __pinocchio_algorithm_geometry_txx__
