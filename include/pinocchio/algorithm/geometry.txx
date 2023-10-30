//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_geometry_txx__
#define __pinocchio_algorithm_geometry_txx__

namespace pinocchio {

  extern template PINOCCHIO_DLLAPI void updateGeometryPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
    (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_DLLAPI void updateGeometryPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, const context::Data &, const GeometryModel &, GeometryData &);

#ifdef PINOCCHIO_WITH_HPP_FCL

  extern template PINOCCHIO_DLLAPI bool computeCollisions
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
    (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &, const bool stopAtFirstCollision);

  extern template PINOCCHIO_DLLAPI std::size_t computeDistances
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
    (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

  extern template PINOCCHIO_DLLAPI void computeBodyRadius
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (const context::Model &, const GeometryModel &, GeometryData &);

#endif // PINOCCHIO_WITH_HPP_FCL

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_geometry_txx__
