//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/collision/distance.hpp"

namespace pinocchio {

  template std::size_t computeDistances
    <context::Scalar, context::Options, JointCollectionDefaultTpl, context::VectorXs>
    (const context::Model &, context::Data &, const GeometryModel &, GeometryData &, const Eigen::MatrixBase<context::VectorXs> &);

} // namespace pinocchio

