//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/algorithm/crba.hpp"

namespace pinocchio {
namespace impl {
  namespace minimal {

    template const context::MatrixXs & crba
      <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
    (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
    } // namespace minimal

  template const context::MatrixXs & crba
    <context::Scalar, context::Options, JointCollectionDefaultTpl, Eigen::Ref<const context::VectorXs>>
  (const context::Model &, context::Data &, const Eigen::MatrixBase<Eigen::Ref<const context::VectorXs>> &);
} // namespace impl
} // namespace pinocchio
