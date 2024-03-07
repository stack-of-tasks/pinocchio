//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/collision/collision.hpp"

namespace pinocchio {

  template bool computeCollisions
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &, const bool stopAtFirstCollision);

  template std::size_t computeDistances
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
  (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

  template void computeBodyRadius
    <context::Scalar, context::Options, JointCollectionDefaultTpl>
  (const context::Model &, const GeometryModel &, GeometryData &);

} // namespace pinocchio
