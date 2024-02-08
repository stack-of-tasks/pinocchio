//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/geometry.hpp"

namespace pinocchio {

  template void updateGeometryPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

  template void updateGeometryPlacements
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const context::Data &, const GeometryModel &, GeometryData &);

#ifdef PINOCCHIO_WITH_HPP_FCL

  template bool computeCollisions
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &, const bool stopAtFirstCollision);

  template std::size_t computeDistances
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

  template void computeBodyRadius
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const GeometryModel &, GeometryData &);

#endif // PINOCCHIO_WITH_HPP_FCL

} // namespace pinocchio
